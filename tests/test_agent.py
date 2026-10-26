import json
import threading
import time
import unittest

from agent import tools
from agent.loop import AgentRunner, extract_json
from robot.estop import EStopSender
from robot.stub import StubRobot

CFG = {"webcam": {"index": 99, "width": 320, "height": 240, "test_image": None},
       "robot": {"pan_home_deg": None},
       "agent": {"max_steps": 6, "max_task_s": 30, "max_parse_retries": 2, "log_dir": "logs/test"}}


class FakeModel:
    """Returns scripted replies; records calls."""
    def __init__(self, replies, delay=0.0):
        self.replies = list(replies)
        self.calls = 0
        self.delay = delay

    def chat(self, messages, cancel_event=None, fmt=None, on_chunk=None):
        self.calls += 1
        end = time.time() + self.delay
        while time.time() < end:
            if cancel_event is not None and cancel_event.is_set():
                return "", "", {"cancelled": True}
            time.sleep(0.01)
        r = self.replies.pop(0) if self.replies else json.dumps({"plan": [{"tool": "finish", "args": {"answer": "x"}}]})
        if on_chunk:
            on_chunk({"type": "thinking", "text": "hmm"})
            on_chunk({"type": "content", "text": r})
        return "hmm", r, {"ttft_s": 0.0}


def plan(*steps):
    return json.dumps({"observation": "o", "plan": [{"tool": t, "args": a} for t, a in steps]})


def make(replies, delay=0.0, tool_delay=0.05):
    robot = StubRobot(CFG, tool_delay_s=tool_delay)
    est = EStopSender(None, local_latch=robot.latch_estop)
    events = []
    r = AgentRunner(CFG, FakeModel(replies, delay), robot, est, on_event=events.append)
    return r, robot, est, events


def wait_idle(r, t=10):
    end = time.time() + t
    while r.busy() and time.time() < end:
        time.sleep(0.02)
    return not r.busy()


class TestRegistry(unittest.TestCase):
    def test_no_reset_or_battery_tool(self):
        for name in tools.TOOLS:
            for bad in ("reset", "clear", "unlatch", "battery"):
                self.assertNotIn(bad, name)
        with self.assertRaises(ValueError):
            tools.check_registry({"reset_estop": {}})
        with self.assertRaises(ValueError):
            tools.check_registry({"battery": {}})

    def test_validate_rejects_unknown_tool_and_bad_args(self):
        with self.assertRaises(Exception):
            tools.validate_step({"plan": [{"tool": "reset_estop", "args": {}}]})
        with self.assertRaises(ValueError):
            tools.validate_step({"plan": [{"tool": "move", "args": {"distance_m": 5}}]})
        tools.validate_step({"plan": [{"tool": "look", "args": {"tilt_deg": 10, "pan": {"mode": "relative", "deg": 20}}}]})

    def test_extract_json(self):
        self.assertEqual(extract_json('```json\n{"plan": []}\n```'), {"plan": []})
        self.assertEqual(extract_json('sure! {"a": 1} done'), {"a": 1})


class TestLoop(unittest.TestCase):
    def test_runs_plan_and_finishes(self):
        r, robot, _, ev = make([plan(("look", {"tilt_deg": 10}), ("finish", {"answer": "ok"}))])
        r.submit("look up")
        self.assertTrue(wait_idle(r))
        types = [e["type"] for e in ev]
        self.assertIn("plan", types)
        self.assertIn("finished", types)
        steps = [e for e in ev if e["type"] == "step" and e["status"] == "done"]
        self.assertTrue(steps[0]["result"]["data"]["stub"])

    def test_two_retry_parse_rule(self):
        r, _, _, ev = make(["garbage", "still garbage", "nope"])
        r.submit("x")
        self.assertTrue(wait_idle(r))
        self.assertEqual(r.model.calls, 3)            # 1 try + 2 retries
        self.assertEqual(len([e for e in ev if e["type"] == "parse_error"]), 3)
        self.assertTrue(any(e["type"] == "failed" for e in ev))

    def test_retry_recovers(self):
        r, _, _, ev = make(["bad", plan(("finish", {"answer": "ok"}))])
        r.submit("x")
        self.assertTrue(wait_idle(r))
        self.assertTrue(any(e["type"] == "finished" for e in ev))

    def test_long_plan_runs_in_order_with_results(self):
        r, _, _, ev = make([plan(*[("snapshot", {})] * 11, ("finish", {"answer": "ok"}))], tool_delay=0.0)
        r.max_steps = None
        r.submit("x")
        self.assertTrue(wait_idle(r))
        p = [e for e in ev if e["type"] == "plan"][0]
        self.assertEqual(len(p["steps"]), 12)
        done = [e for e in ev if e["type"] == "step" and e["status"] == "done"]
        self.assertGreaterEqual(len(done), 11)
        self.assertTrue(all("result" in e for e in done))
        self.assertTrue(any(e["type"] == "finished" for e in ev))

    def test_step_limit(self):
        r, _, _, ev = make([plan(*[("stop", {})] * 8)] * 3, tool_delay=0.0)
        r.submit("x")
        self.assertTrue(wait_idle(r))
        self.assertTrue(any(e["type"] == "limit" for e in ev))

    def test_cancel_single_tool(self):
        r, _, _, ev = make([plan(("move", {"distance_m": 0.5}), ("finish", {"answer": "ok"}))], tool_delay=2.0)
        r.submit("x")
        sid = None
        end = time.time() + 5
        while sid is None and time.time() < end:
            for e in list(ev):
                if e["type"] == "step" and e["status"] == "running":
                    sid = e["step_id"]
            time.sleep(0.02)
        r.cancel_step(sid)
        self.assertTrue(wait_idle(r))
        self.assertTrue(any(e["type"] == "step" and e["status"] == "cancelled" for e in ev))

    def test_cancel_whole_run_during_inference(self):
        r, _, _, ev = make([plan(("finish", {"answer": "ok"}))], delay=3.0)
        r.submit("x")
        time.sleep(0.3)
        t = time.time()
        r.cancel_run()
        self.assertTrue(wait_idle(r, 3))
        self.assertLess(time.time() - t, 1.5)

    def test_human_command_pauses_and_resumes(self):
        replies = [plan(("move", {"distance_m": 0.3}), ("finish", {"answer": "task"})),
                   plan(("finish", {"answer": "human"})),
                   plan(("finish", {"answer": "resumed"}))]
        r, _, _, ev = make(replies, tool_delay=1.0)
        r.submit("original task")
        time.sleep(0.4)
        self.assertEqual(r.submit("human: look left"), "queued_as_human_command")
        self.assertTrue(wait_idle(r))
        answers = [e["answer"] for e in ev if e["type"] == "finished"]
        self.assertEqual(answers, ["human", "resumed"])
        self.assertTrue(any(e["type"] == "human_done" for e in ev))


class TestEStop(unittest.TestCase):
    def test_estop_fires_while_model_busy_and_latches(self):
        r, robot, est, ev = make([plan(("finish", {"answer": "ok"}))], delay=5.0)
        r.submit("x")
        time.sleep(0.2)
        t = time.time()
        est.trigger("gui")
        while not robot.estop_latched and time.time() - t < 1:
            time.sleep(0.005)
        self.assertTrue(robot.estop_latched)
        self.assertLess(time.time() - t, 0.5)       # did not wait for the 5 s "inference"
        r.cancel_run()
        wait_idle(r)

    def test_agent_estop_tool_latches_and_motion_refused(self):
        r, robot, _, ev = make([plan(("estop", {"reason": "test"}), ("move", {"distance_m": 0.2}))] * 3)
        r.submit("x")
        time.sleep(1.0)
        self.assertTrue(robot.estop_latched)
        self.assertEqual(r.state, "paused_estop")
        r.resume()                                   # Resume must NOT clear the latch
        time.sleep(0.5)
        self.assertTrue(robot.estop_latched)
        self.assertEqual(r.state, "paused_estop")
        r.cancel_run()
        wait_idle(r)
        self.assertTrue(robot.estop_latched)

    def test_no_code_path_can_clear(self):
        import inspect
        import robot.estop as m1, robot.stub as m2, robot.bridge_client as m3, agent.loop as m4
        for mod in (m1, m2, m3, m4):
            src = inspect.getsource(mod)
            self.assertNotIn("estop_latched = False", src)
            self.assertNotIn("/reset", src)
        self.assertFalse(any(n for n in dir(EStopSender) if "reset" in n or "clear" in n))
        r = StubRobot(CFG)
        self.assertFalse(any(n for n in dir(r) if "unlatch" in n or "reset" in n))


if __name__ == "__main__":
    unittest.main()


class TestHardStop(unittest.TestCase):
    def test_hard_stop_cancels_and_sends_stop_without_latching(self):
        r, robot, _, ev = make([plan(("move", {"distance_m": 0.5}), ("finish", {"answer": "ok"}))], tool_delay=3.0)
        r.submit("x")
        time.sleep(0.4)
        t = time.time()
        res = r.hard_stop("halt")
        self.assertTrue(res["ok"])
        self.assertTrue(wait_idle(r, 2))
        self.assertLess(time.time() - t, 1.5)
        self.assertFalse(robot.estop_latched)
        self.assertTrue(any(e["type"] == "hard_stop" for e in ev))
