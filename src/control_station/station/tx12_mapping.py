"""Read the TX12 control-station display map.

This file describes physical controls and their observed EdgeTX output channels.
It does not configure the transmitter or command the buoy.
"""

from __future__ import annotations

import json
from pathlib import Path


DEFAULT_TX12_MAPPING_PATH = Path(__file__).resolve().parents[1] / "config" / "tx12-channel-map.json"


def load_tx12_mapping(path: Path = DEFAULT_TX12_MAPPING_PATH) -> list[dict[str, object]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("TX12 map must use schema_version 1")
    source = document.get("controls")
    if not isinstance(source, list) or not source:
        raise ValueError("TX12 map needs a nonempty controls list")

    controls: list[dict[str, object]] = []
    seen_ids: set[str] = set()
    for index, item in enumerate(source):
        if not isinstance(item, dict):
            raise ValueError(f"TX12 control {index + 1} must be an object")
        control_id = item.get("id")
        label = item.get("label")
        kind = item.get("kind")
        channel = item.get("channel")
        if not isinstance(control_id, str) or not control_id or control_id in seen_ids:
            raise ValueError(f"TX12 control {index + 1} needs a unique id")
        if not isinstance(label, str) or not label:
            raise ValueError(f"TX12 control {control_id} needs a label")
        if kind not in ("rf_axis", "rf_switch", "radio_ui"):
            raise ValueError(f"TX12 control {control_id} has an unknown kind")
        if channel is not None and (type(channel) is not int or not 1 <= channel <= 16):
            raise ValueError(f"TX12 control {control_id} channel must be 1–16 or null")
        if kind == "radio_ui" and channel is not None:
            raise ValueError(f"TX12 local control {control_id} cannot have an RF channel")
        purpose = item.get("purpose", "")
        if not isinstance(purpose, str):
            raise ValueError(f"TX12 control {control_id} purpose must be text")
        positions = item.get("positions", [])
        if kind == "rf_switch":
            if not isinstance(positions, list) or len(positions) not in (2, 3):
                raise ValueError(f"TX12 switch {control_id} needs two or three positions")
            for position in positions:
                if not isinstance(position, dict) or not isinstance(position.get("label"), str) or not position["label"]:
                    raise ValueError(f"TX12 switch {control_id} position needs a label")
                meaning = position.get("meaning")
                if meaning is not None and (not isinstance(meaning, str) or not meaning.strip()):
                    raise ValueError(f"TX12 switch {control_id} position meaning must be text or null")
        elif positions:
            raise ValueError(f"TX12 control {control_id} cannot define switch positions")
        image_x, image_y = item.get("image_x"), item.get("image_y")
        if kind != "radio_ui" and (
            not isinstance(image_x, (int, float)) or not 0 <= image_x <= 1
            or not isinstance(image_y, (int, float)) or not 0 <= image_y <= 1
        ):
            raise ValueError(f"TX12 control {control_id} needs image_x/image_y in 0–1")
        seen_ids.add(control_id)
        controls.append({
            "id": control_id,
            "label": label,
            "kind": kind,
            "channel": channel,
            "purpose": purpose,
            "positions": [{"label": position["label"], "meaning": position.get("meaning")} for position in positions],
            "image_x": image_x if kind != "radio_ui" else None,
            "image_y": image_y if kind != "radio_ui" else None,
        })
    return controls
