import json
import tempfile
import threading
import time
import unittest
from pathlib import Path

from agent import tools
from agent.skills import Skills, valid_bbox
from robot.stub import StubRobot
from tests.test_agent import FakeModel, plan, wait_idle
from agent.loop import AgentRunner
from robot.estop import EStopSender

TMP = tempfile.mkdtemp()


def cfg():
    return {"webcam": {"index": 99, "width": 320, "height": 240, "test_image": None},
            "robot": {"pan_home_deg": None},
            "agent": {"max_steps": None, "max_task_s": 30, "max_parse_retries": 2, "log_dir": "logs/test",
                      "locations_file": str(Path(tempfile.mkdtemp()) / "loc.json"), "record_dir": TMP}}


def seen(found, bbox=None):
    return json.dumps({"found": found, "description": "d", "bbox": bbox})


def mk(replies=()):
    c = cfg()
    robot = StubRobot(c, tool_delay_s=0.0)
    calls = []
    orig = robot.call
    def rec(tool, args, cid, cancel, timeout_s=30):
        calls.append((tool, args))
        return orig(tool, args, cid, cancel, timeout_s)
    robot.call = rec
    s = Skills(c, FakeModel(list(replies)), robot)
    s.interval_scale = 0.01
    return s, robot, calls


class TestSkills(unittest.TestCase):
    def test_bbox_validation(self):
        self.assertEqual(valid_bbox([10, 20, 50, 60], 320, 240), [10, 20, 50, 60])
        self.assertEqual(valid_bbox([50, 20, 10, 999], 320, 240), [10, 20, 50, 240])
        self.assertIsNone(valid_bbox([1, 2, 3], 320, 240))
        self.assertIsNone(valid_bbox("x", 320, 240))

    def test_describe_scene(self):
        s, _, _ = mk([seen(True, [10, 10, 100, 100])])
        r = s.run("describe_scene", {"target": "cup"}, threading.Event())
        self.assertTrue(r["ok"]); self.assertTrue(r["data"]["found"]); self.assertEqual(r["data"]["bbox"], [10, 10, 100, 100])
        s, _, _ = mk(["not json at all"])
        r = s.run("describe_scene", {}, threading.Event())
        self.assertIsNone(r["data"]["found"])

    def test_feed_and_watch_for(self):
        s, _, _ = mk([seen(False), seen(False), seen(True, [0, 0, 50, 50])])
        r = s.run("feed", {"goal": "cup", "interval_s": 0.5, "max_s": 100}, threading.Event())
        self.assertTrue(r["data"]["met"]); self.assertEqual(r["data"]["frames"], 3)
        s, _, _ = mk([seen(False)] * 100)
        r = s.run("watch_for", {"goal": "cup", "max_s": 3}, threading.Event())
        self.assertFalse(r["data"]["met"]); self.assertEqual(r["data"]["reason"], "timeout")

    def test_cancel(self):
        s, _, _ = mk([seen(False)] * 1000)
        ev = threading.Event(); ev.set()
        self.assertEqual(s.run("watch_for", {"goal": "x"}, ev)["error"]["code"], "CANCELLED")
        self.assertEqual(s.run("wait", {"seconds": 100}, ev)["error"]["code"], "CANCELLED")

    def test_wait(self):
        s, _, _ = mk()
        self.assertTrue(s.run("wait", {"seconds": 1}, threading.Event())["ok"])

    def test_exploring(self):
        s, _, calls = mk([seen(False), seen(True, [0, 0, 9, 9])])
        r = s.run("start_exploring", {"until_seen": "door", "max_s": 100}, threading.Event())
        self.assertEqual(r["data"]["stopped_because"], "seen")
        self.assertEqual(calls[0], ("map", {"action": "explore", "enable": True, "max_s": 100}))
        self.assertEqual(calls[-1], ("map", {"action": "explore", "enable": False}))
        s, _, calls = mk()
        self.assertTrue(s.run("start_exploring", {}, threading.Event())["data"]["exploring"])
        self.assertTrue(s.run("stop_exploring", {}, threading.Event())["ok"])
        self.assertEqual(calls[-1][1]["enable"], False)

    def test_save_map(self):
        s, _, calls = mk()
        self.assertEqual(s.run("save_map", {"name": "lab"}, threading.Event())["data"]["saved"], "lab")
        self.assertEqual(calls[-1], ("map", {"action": "save", "name": "lab"}))

    def test_locations(self):
        s, robot, calls = mk()
        robot.pose = {"x": 1.5, "y": -2.0, "yaw": 0.3, "frame": "map"}
        ev = threading.Event()
        self.assertEqual(s.run("save_location", {"name": "kitchen"}, ev)["data"]["saved"]["x"], 1.5)
        s.run("note_location", {"name": "kitchen", "note": "fridge on the left"}, ev)
        r = s.run("note_location", {"note": "charging cable here"}, ev)
        self.assertEqual(r["data"]["saved"]["name"], "spot-1")
        locs = s.run("list_locations", {}, ev)["data"]["locations"]
        self.assertEqual({l["name"] for l in locs}, {"kitchen", "spot-1"})
        self.assertIn("fridge", s.store.brief())
        robot.pose = {"x": 0, "y": 0, "yaw": 0, "frame": "map"}
        r = s.run("recall_location", {"name": "kitchen"}, ev)
        self.assertTrue(r["ok"]); self.assertEqual(calls[-1][0], "goto_pose"); self.assertEqual(calls[-1][1]["x"], 1.5)
        self.assertEqual(s.run("recall_location", {"name": "nope"}, ev)["error"]["code"], "UNKNOWN_LOCATION")
        self.assertEqual(s.run("return_home", {}, ev)["error"]["code"], "UNKNOWN_LOCATION")
        s.run("save_location", {"name": "home"}, ev)
        self.assertTrue(s.run("return_home", {}, ev)["ok"])
        self.assertTrue(s.run("delete_location", {"name": "kitchen"}, ev)["ok"])
        self.assertFalse(s.run("delete_location", {"name": "kitchen"}, ev)["ok"])

    def test_scan_area(self):
        s, _, calls = mk([seen(False), seen(True, [0, 0, 9, 9])])
        r = s.run("scan_area", {"target": "cup", "angles_deg": [-30, 30, 60]}, threading.Event())
        self.assertTrue(r["data"]["found"]); self.assertEqual(r["data"]["at_deg"], 30)
        self.assertEqual(calls[1][1]["pan"]["deg"], 60)
        s, _, calls = mk([seen(False)] * 3)
        r = s.run("scan_area", {"target": "cup", "use": "body", "angles_deg": [90, 90, 90]}, threading.Event())
        self.assertFalse(r["data"]["found"]); self.assertEqual([c[0] for c in calls], ["move"] * 3)

    def test_go_to_object_and_follow(self):
        s, _, calls = mk([seen(True, [0, 100, 40, 140]), seen(True, [140, 50, 180, 230])])
        r = s.run("go_to_object", {"target": "chair", "max_m": 2}, threading.Event())
        self.assertTrue(r["data"]["reached"]); self.assertGreater(calls[0][1]["turn_deg"], 0)
        s, _, _ = mk([seen(True, [0, 0, 10, 10]), seen(False)])
        r = s.run("follow", {"target": "person", "max_s": 100}, threading.Event())
        self.assertEqual(r["data"]["reason"], "lost")

    def test_motion_refused_when_latched(self):
        s, robot, calls = mk([seen(True, [0, 0, 10, 10])] * 5)
        robot.latch_estop()
        for t, a in [("start_exploring", {}), ("recall_location", {"name": "x"}), ("go_to_object", {"target": "a"}),
                     ("follow", {"target": "a"}), ("scan_area", {"target": "a"}), ("return_home", {})]:
            self.assertEqual(s.run(t, a, threading.Event())["error"]["code"], "ESTOP_LATCHED", t)
        self.assertFalse(any(c[0] in ("move", "goto_pose", "map", "look") for c in calls))
        self.assertTrue(robot.estop_latched)   # nothing clears it

    def test_set_speed_stub(self):
        s, robot, _ = mk()
        r = robot.call("set_speed", {"max_mps": 0.3}, "c", None)
        self.assertTrue(r["ok"]); self.assertEqual(r["data"]["max_mps"], 0.3)

    def test_record_video(self):
        s, _, _ = mk()
        ev = threading.Event()
        r = s.run("record_video", {"action": "start", "fps": 10}, ev)
        self.assertTrue(r["data"]["recording"])
        time.sleep(0.5)
        r = s.run("record_video", {"action": "stop"}, ev)
        self.assertGreater(r["data"]["frames"], 0)


class TestRegistryNew(unittest.TestCase):
    def test_every_tool_has_outputs_and_no_forbidden(self):
        for n, s in tools.TOOLS.items():
            self.assertTrue(s.get("outputs"), n)
        for bad in ("battery", "dock", "charge", "speak"):
            self.assertFalse(any(bad in n for n in tools.TOOLS))

    def test_tools_md_up_to_date(self):
        from scripts.gen_tools_md import render
        p = Path(__file__).resolve().parents[1] / "TOOLS.md"
        self.assertTrue(p.exists(), "TOOLS.md missing: run python scripts/gen_tools_md.py")
        self.assertEqual(p.read_text(encoding="utf-8"), render(),
                         "TOOLS.md is out of date: run python scripts/gen_tools_md.py and commit it")


class TestLoopIntegration(unittest.TestCase):
    def test_loop_runs_skill_and_injects_locations(self):
        c = cfg()
        robot = StubRobot(c, tool_delay_s=0.0)
        model = FakeModel([plan(("save_location", {"name": "desk", "note": "my desk"}),
                                ("describe_scene", {"target": "cup"})),
                           seen(True, [1, 1, 20, 20]),
                           plan(("finish", {"answer": "ok"}))])
        seen_msgs = []
        orig = model.chat
        def chat(messages, **kw):
            seen_msgs.append(json.dumps([m.get("content") for m in messages]))
            return orig(messages, **kw)
        model.chat = chat
        ev = []
        r = AgentRunner(c, model, robot, EStopSender(None, local_latch=robot.latch_estop), on_event=ev.append)
        r.submit("find cup")
        self.assertTrue(wait_idle(r))
        self.assertTrue(any(e["type"] == "finished" for e in ev))
        done = [e for e in ev if e["type"] == "step" and e["status"] == "done"]
        self.assertEqual([e["tool"] for e in done][:2], ["save_location", "describe_scene"])
        r.submit("again")
        self.assertTrue(wait_idle(r))
        self.assertIn("Saved locations: desk", seen_msgs[-1])
        self.assertNotIn("laptop", seen_msgs[0].split("Tools:")[0].lower())
