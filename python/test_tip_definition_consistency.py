"""Regression tests for the Gen3 Lite 2F tool-tip definition."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from mujoco_backend_core import HeadlessMujocoBackend
from mujoco_scene_utils import compute_tip_world
from online_config_utils import initial_q_from_payload, normalize_online_config_payload


def test_legacy_opposite_finger_offsets_are_upgraded() -> None:
    """Existing online JSON files with the old +/- offsets must normalize."""

    payload = _load_smoke_payload()
    assert payload["model_kwargs"]["tip_left_local"] == [0.0, 0.0, 0.0]
    assert payload["model_kwargs"]["tip_right_local"] == [0.0, 0.0, 0.0]
    assert payload["backend_cfg"]["tip_left_local"] == [0.0, 0.0, 0.0]
    assert payload["backend_cfg"]["tip_right_local"] == [0.0, 0.0, 0.0]


def test_mujoco_tip_is_distal_finger_origin_midpoint() -> None:
    """The visual/control tip should sit at the gripper-center midpoint."""

    payload = _load_smoke_payload()
    controller_cfg = payload["controller_cfg"]
    backend = HeadlessMujocoBackend(payload["backend_cfg"], controller_cfg)
    q0 = initial_q_from_payload(payload)
    backend.reset(q0, np.zeros_like(q0))
    left_origin = _body_origin(backend.model, backend.data, "LEFT_FINGER_DIST")
    right_origin = _body_origin(backend.model, backend.data, "RIGHT_FINGER_DIST")
    expected_tip = 0.5 * (left_origin + right_origin)
    actual_tip = compute_tip_world(backend.model, backend.data, backend.backend_cfg)
    np.testing.assert_allclose(actual_tip, expected_tip, atol=1e-12, rtol=0.0)


def _load_smoke_payload() -> dict:
    """Load and normalize the standard online smoke config."""

    repo_root = Path(__file__).resolve().parent.parent
    config_path = repo_root / "results" / "online_config" / "routeb_online_config_manual_smoke.json"
    return normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=repo_root,
    )


def _body_origin(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> np.ndarray:
    """Resolve optional MuJoCo arm_ prefix and return body origin."""

    for candidate in (body_name, "arm_" + body_name):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, candidate)
        if body_id >= 0:
            return np.asarray(data.xpos[body_id], dtype=float).reshape(3).copy()
    raise AssertionError(f"MuJoCo body not found: {body_name}")
