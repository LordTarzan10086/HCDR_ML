"""Pinocchio planar root + serial arm model builder for Route-B.

This module provides a compact reference implementation used by pin_terms.py.
"""

from __future__ import annotations

from typing import Iterable, Optional

import numpy as np

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover
    raise ImportError("pinocchio is required to build the model.") from exc


def build_planar_root_arm_model(
    n_m: int = 6,
    link_lengths: Optional[Iterable[float]] = None,
    micro_g: bool = True,
):
    """Build a planar-root (x,y,yaw) + revolute-arm model."""

    if n_m < 1:
        raise ValueError("n_m must be >= 1")

    if link_lengths is None:
        link_lengths = 0.25 * np.ones(n_m)
    link_lengths = np.asarray(list(link_lengths), dtype=float).reshape(-1)
    if link_lengths.shape[0] != n_m:
        raise ValueError("link_lengths size must equal n_m")

    model = pin.Model()
    if micro_g:
        model.gravity.linear[:] = 0.0

    # Planar root: x-y translation + yaw rotation.
    jid_x = model.addJoint(0, pin.JointModelPX(), pin.SE3.Identity(), "root_px")
    model.appendBodyToJoint(jid_x, pin.Inertia.Random(), pin.SE3.Identity())

    jid_y = model.addJoint(jid_x, pin.JointModelPY(), pin.SE3.Identity(), "root_py")
    model.appendBodyToJoint(jid_y, pin.Inertia.Random(), pin.SE3.Identity())

    jid_yaw = model.addJoint(jid_y, pin.JointModelRZ(), pin.SE3.Identity(), "root_yaw")
    model.appendBodyToJoint(jid_yaw, pin.Inertia.Random(), pin.SE3.Identity())

    parent = jid_yaw
    for i, l in enumerate(link_lengths, start=1):
        joint_name = f"joint_{i}"
        jid = model.addJoint(parent, pin.JointModelRZ(), pin.SE3.Identity(), joint_name)
        com = np.array([0.5 * float(l), 0.0, 0.0], dtype=float)
        inertia = pin.Inertia.FromSphere(1.0, 0.02)
        inertia.lever[:] = com
        model.appendBodyToJoint(jid, inertia, pin.SE3.Identity())
        placement = pin.SE3(np.eye(3), np.array([float(l), 0.0, 0.0], dtype=float))
        model.addFrame(pin.Frame(f"link_{i}", jid, jid, placement, pin.FrameType.BODY))
        parent = jid

    return model
