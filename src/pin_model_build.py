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
    """Build a planar-root (x,y,yaw) + revolute-arm Pinocchio model.

    Args:
        n_m: Number of arm joints.
        link_lengths: Iterable of link lengths in meters, length n_m.
        micro_g: If True, set gravity vector to zero.

    Returns:
        pin.Model configured as planar root + serial RZ arm chain.
    """

    # Validate requested model size.
    if n_m < 1:
        raise ValueError("n_m must be >= 1")

    # Normalize and validate link-length vector [m], shape (n_m,).
    if link_lengths is None:
        link_lengths = 0.25 * np.ones(n_m)
    link_lengths = np.asarray(list(link_lengths), dtype=float).reshape(-1)
    if link_lengths.shape[0] != n_m:
        raise ValueError("link_lengths size must equal n_m")

    # Create empty model and optionally disable gravity for micro-g mode.
    model = pin.Model()
    if micro_g:
        model.gravity.linear[:] = 0.0

    # Planar root: x-y translation + yaw rotation.
    rootJointIdX = model.addJoint(0, pin.JointModelPX(), pin.SE3.Identity(), "root_px")
    model.appendBodyToJoint(rootJointIdX, pin.Inertia.Random(), pin.SE3.Identity())

    rootJointIdY = model.addJoint(rootJointIdX, pin.JointModelPY(), pin.SE3.Identity(), "root_py")
    model.appendBodyToJoint(rootJointIdY, pin.Inertia.Random(), pin.SE3.Identity())

    rootJointIdYaw = model.addJoint(rootJointIdY, pin.JointModelRZ(), pin.SE3.Identity(), "root_yaw")
    model.appendBodyToJoint(rootJointIdYaw, pin.Inertia.Random(), pin.SE3.Identity())

    # Append serial RZ arm joints and associated body frames.
    parentJointId = rootJointIdYaw
    for jointIndex, linkLengthM in enumerate(link_lengths, start=1):
        jointName = f"joint_{jointIndex}"
        jointId = model.addJoint(parentJointId, pin.JointModelRZ(), pin.SE3.Identity(), jointName)

        # Approximate each link as a sphere inertia shifted along +x.
        linkComOffsetM = np.array([0.5 * float(linkLengthM), 0.0, 0.0], dtype=float)
        linkInertia = pin.Inertia.FromSphere(1.0, 0.02)
        linkInertia.lever[:] = linkComOffsetM
        model.appendBodyToJoint(jointId, linkInertia, pin.SE3.Identity())

        # Add body frame at link distal end for forward-kinematics references.
        distalPlacement = pin.SE3(
            np.eye(3), np.array([float(linkLengthM), 0.0, 0.0], dtype=float)
        )
        model.addFrame(
            pin.Frame(f"link_{jointIndex}", jointId, jointId, distalPlacement, pin.FrameType.BODY)
        )
        parentJointId = jointId

    return model
