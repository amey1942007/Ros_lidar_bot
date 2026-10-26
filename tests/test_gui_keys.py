"""F11 / Esc only change fullscreen; they must never trigger or clear the E-STOP."""
import unittest

from config import load_config


class TestGuiKeys(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from gui import app as guiapp
        cls.guiapp = guiapp
        cls.root = guiapp.make_root()
        cls.app = guiapp.App(cls.root, load_config())
        cls.root.update()

    @classmethod
    def tearDownClass(cls):
        cls.root.destroy()

    def _press(self, key):
        self.root.focus_force()
        self.root.event_generate(f"<{key}>", when="tail")
        self.root.update()

    def test_keys_never_touch_estop(self):
        calls = []
        orig = self.app.estop.trigger
        self.app.estop.trigger = lambda *a, **k: calls.append(1)
        try:
            for latched in (False, True):
                self.app.robot.estop_latched = latched
                for key in ("F11", "Escape", "F11", "F11", "Escape", "Escape"):
                    self._press(key)
                    self.assertEqual(self.app.robot.estop_latched, latched, key)
                self.win_handlers_direct(latched)
            self.assertEqual(calls, [], "F11/Esc triggered the E-STOP")
        finally:
            self.app.estop.trigger = orig
            self.root.attributes("-fullscreen", False)

    def win_handlers_direct(self, latched):
        w = self.app.win
        w.toggle_fullscreen(); self.assertTrue(w.fullscreen)
        w.exit_fullscreen(); self.assertFalse(w.fullscreen)
        self.assertEqual(self.app.robot.estop_latched, latched)

    def test_bindings_point_at_window_control_only(self):
        for key in ("<F11>", "<Escape>"):
            script = self.root.bind(key)
            self.assertNotIn("_press_estop", script)
        self.assertFalse(any(hasattr(self.app.win, n) for n in ("estop", "robot", "runner")))


if __name__ == "__main__":
    unittest.main()
