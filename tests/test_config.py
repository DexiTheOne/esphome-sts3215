"""Configuration boundaries for normal controls and explicit ID provisioning.

Run with the ESPHome environment: python -m unittest discover -s tests
"""

import importlib.util
from pathlib import Path
import unittest

import esphome.config_validation as cv
from esphome.core import CORE


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("sts3215_config", ROOT / "components/sts3215/__init__.py")
component = importlib.util.module_from_spec(spec)
spec.loader.exec_module(component)


class ConfigTests(unittest.TestCase):
    def setUp(self):
        CORE.reset()

    def panel(self):
        return {
            "current_id": {"name": "Current ID"},
            "new_id": {"name": "New ID"},
            "set_id": {"name": "Set ID"},
        }

    def test_multiple_servos(self):
        config = component.CONFIG_SCHEMA({"servos": [{"servo_id": 1}, {"servo_id": 2}]})
        self.assertEqual(len(config["servos"]), 2)

    def test_movement_modes(self):
        base = {"servos": [{"servo_id": 1}, {"servo_id": 2}]}
        self.assertEqual(component.CONFIG_SCHEMA(base)["movement_mode"], "staggered")
        for mode in ("staggered", "overlapping"):
            config = component.CONFIG_SCHEMA({**base, "movement_mode": mode, "start_delay": "750ms"})
            self.assertEqual(config["movement_mode"], mode)
            self.assertEqual(config["start_delay"].total_milliseconds, 750)
        with self.assertRaises(cv.Invalid):
            component.CONFIG_SCHEMA({**base, "movement_mode": "unknown"})

    def test_duplicate_servo_ids(self):
        with self.assertRaises(cv.Invalid):
            component.CONFIG_SCHEMA({"servos": [{"servo_id": 1}, {"servo_id": 1}]})

    def test_panel_without_servos(self):
        config = component.CONFIG_SCHEMA({"provisioning": self.panel()})
        self.assertEqual(config["servos"], [])
        self.assertEqual(config["provisioning"]["current_id"]["id"].type, component.sensor.Sensor)

    def test_current_id_rejects_editable_number_options(self):
        panel = self.panel()
        panel["current_id"]["mode"] = "box"
        with self.assertRaises(cv.Invalid):
            component.CONFIG_SCHEMA({"provisioning": panel})

    def test_empty_normal_component(self):
        with self.assertRaises(cv.Invalid):
            component.CONFIG_SCHEMA({"servos": []})

    def test_panel_and_servo_controls_are_exclusive(self):
        with self.assertRaises(cv.Invalid):
            component.CONFIG_SCHEMA({"provisioning": self.panel(), "servos": [{"servo_id": 1}]})

    def test_group_requires_servos(self):
        with self.assertRaises(cv.Invalid):
            component.CONFIG_SCHEMA({"provisioning": self.panel(), "main_cover": {"name": "All"}})

    def test_action_id_boundaries(self):
        component.SET_ID_ACTION_SCHEMA({"id": "bus", "current_id": 0, "new_id": 253})
        for key in ("current_id", "new_id"):
            for value in (-1, 254, 255, 256, 1.5):
                with self.subTest(key=key, value=value), self.assertRaises(cv.Invalid):
                    component.SET_ID_ACTION_SCHEMA({"id": "bus", "current_id": 1, "new_id": 2, key: value})


if __name__ == "__main__":
    unittest.main()
