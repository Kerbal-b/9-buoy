"""Validation of the station-only TX12 display mapping."""

import json
import unittest
from unittest.mock import patch

from station.tx12_mapping import DEFAULT_TX12_MAPPING_PATH, load_tx12_mapping


class Tx12MappingTests(unittest.TestCase):
    def test_starter_map_uses_mode_2_aetr(self) -> None:
        controls = load_tx12_mapping()
        self.assertEqual(len(controls), 12)
        channels = {item["id"]: item["channel"] for item in controls}
        self.assertEqual(channels["right_stick_horizontal"], 1)
        self.assertEqual(channels["right_stick_vertical"], 2)
        self.assertEqual(channels["left_stick_vertical"], 3)
        self.assertEqual(channels["left_stick_horizontal"], 4)
        self.assertEqual({key: channels[key] for key in ("se", "sf", "sb", "sc", "sa", "sd")},
                         {"se": 5, "sf": 6, "sb": 7, "sc": 8, "sa": 9, "sd": 10})
        self.assertIsNone(channels["s1"])
        self.assertIsNone(channels["s2"])
        self.assertEqual(len(next(item for item in controls if item["id"] == "se")["positions"]), 3)
        self.assertEqual(len(next(item for item in controls if item["id"] == "sa")["positions"]), 2)
        self.assertEqual(len({item["id"] for item in controls}), len(controls))

    def test_user_can_assign_an_rf_channel_but_not_a_menu_button(self) -> None:
        document = json.loads(DEFAULT_TX12_MAPPING_PATH.read_text(encoding="utf-8"))
        document["controls"][0]["channel"] = 6
        with patch.object(type(DEFAULT_TX12_MAPPING_PATH), "read_text", return_value=json.dumps(document)):
            self.assertEqual(load_tx12_mapping()[0]["channel"], 6)

        document["controls"].append({"id": "menu", "label": "Menu", "kind": "radio_ui", "channel": 5})
        with patch.object(type(DEFAULT_TX12_MAPPING_PATH), "read_text", return_value=json.dumps(document)):
            with self.assertRaisesRegex(ValueError, "cannot have an RF channel"):
                load_tx12_mapping()


if __name__ == "__main__":
    unittest.main()
