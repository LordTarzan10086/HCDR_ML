"""Regression check: native MuJoCo backend must use 8 true 3D cable sites."""

from __future__ import annotations

import json
import pathlib
import sys

import mujoco

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(REPO_ROOT / "python") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "python"))

from mujoco_native_hcdr_model import build_native_hcdr_model
from online_config_utils import normalize_online_config_payload


def main() -> None:
    config_path = REPO_ROOT / "results" / "online_config" / "routeb_online_config_manual_smoke.json"
    payload = normalize_online_config_payload(
        json.loads(config_path.read_text(encoding="utf-8")),
        repo_root=config_path.resolve().parent.parent.parent,
    )
    controller_cfg = payload["controller_cfg"]
    model, _data, _meta = build_native_hcdr_model(payload["backend_cfg"], controller_cfg)

    tendon_count = int(controller_cfg["n_c"])
    assert int(model.ntendon) >= tendon_count, f"Expected at least {tendon_count} tendons, got {model.ntendon}"

    wrapped_site_names: list[str] = []
    for tendon_index in range(tendon_count):
        wrap_start = int(model.tendon_adr[tendon_index])
        wrap_count = int(model.tendon_num[tendon_index])
        assert wrap_count == 2, f"Tendon {tendon_index + 1} expected 2 wrap sites, got {wrap_count}"
        for wrap_index in range(wrap_start, wrap_start + wrap_count):
            wrap_type = int(model.wrap_type[wrap_index])
            wrap_objid = int(model.wrap_objid[wrap_index])
            assert wrap_type == int(mujoco.mjtWrap.mjWRAP_SITE), (
                f"Tendon {tendon_index + 1} wrap {wrap_index} is not a site wrap: {wrap_type}"
            )
            site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, wrap_objid)
            wrapped_site_names.append(str(site_name))

    assert all("_planar_" not in site_name for site_name in wrapped_site_names), wrapped_site_names
    assert len([name for name in wrapped_site_names if name.startswith("proximal_anchor_")]) == tendon_count
    assert len([name for name in wrapped_site_names if name.startswith("distal_attach_")]) == tendon_count
    print("ok")


if __name__ == "__main__":
    main()
