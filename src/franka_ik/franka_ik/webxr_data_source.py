"""
WebXR data source — reads controller poses from /tmp/pico_data.json
written by pico_controller_server.py.

Provides TrackerData + button state for franka_pico_input_node.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

DATA_FILE = "/tmp/pico_data.json"


@dataclass
class TrackerData:
    serial_number: str = ""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    is_valid: bool = False
    timestamp: float = 0.0


class WebXRDataSource:
    """Reads /tmp/pico_data.json and returns TrackerData for left/right controllers."""

    def __init__(self):
        self._cache: Optional[dict] = None

    def initialize(self) -> bool:
        return self._read_file() is not None

    def is_available(self) -> bool:
        return self._read_file() is not None

    def _read_file(self) -> Optional[dict]:
        try:
            with open(DATA_FILE, "r") as f:
                data = json.load(f)
            if data.get("connected", False):
                self._cache = data
                return data
            return None
        except (FileNotFoundError, json.JSONDecodeError):
            return None

    def get_tracker_data(self) -> List[TrackerData]:
        data = self._read_file()
        if data is None:
            return []

        trackers = []
        ts = data.get("timestamp", time.time())

        for side, sn in [("left_ctrl_pose", "WEBXR_LEFT"), ("right_ctrl_pose", "WEBXR_RIGHT")]:
            ctrl = data.get(side, {})
            pos = ctrl.get("position", {})
            ori = ctrl.get("orientation", {})

            p = np.array([pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)])
            q = np.array([ori.get("x", 0.0), ori.get("y", 0.0), ori.get("z", 0.0), ori.get("w", 1.0)])

            is_valid = np.linalg.norm(p) > 0.01

            trackers.append(TrackerData(
                serial_number=sn,
                position=p,
                orientation=q,
                is_valid=is_valid,
                timestamp=ts,
            ))

        return trackers

    def get_buttons(self) -> Dict[str, List[dict]]:
        """Return button states: {'left': [...], 'right': [...]}"""
        data = self._cache
        if data is None:
            data = self._read_file()
        if data is None:
            return {"left": [], "right": []}
        return {
            "left": data.get("left_buttons", []),
            "right": data.get("right_buttons", []),
        }

    def close(self):
        pass
