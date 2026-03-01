"""Pinocchio dynamics terms for Route-B: M via CRBA, h via RNEA(qdd=0)."""

from __future__ import annotations

from typing import Iterable, Optional, Tuple

import numpy as np
import pinocchio as pin

from pin_model_build import build_planar_root_arm_model

_MODEL = None
_DATA = None


def ensure_model(n_m: int = 6, link_lengths: Optional[Iterable[float]] = None):
    """Create and cache Pinocchio model/data once, then reuse.

    Args:
        n_m: Number of arm joints.
        link_lengths: Optional link-length iterable [m], length n_m.

    Returns:
        Tuple (model, data) for Pinocchio dynamics calls.
    """

    global _MODEL, _DATA

    # Lazily build model/data to avoid rebuilding for each call.
    if _MODEL is None:
        _MODEL = build_planar_root_arm_model(n_m=n_m, link_lengths=link_lengths, micro_g=True)
        _DATA = _MODEL.createData()
    return _MODEL, _DATA


def get_M_h(q: Iterable[float], qd: Iterable[float]) -> Tuple[np.ndarray, np.ndarray]:
    """Return inertia matrix M and bias vector h for Route-B dynamics.

    Args:
        q: Generalized coordinates, shape (n_q,).
        qd: Generalized velocities, shape (n_q,).

    Returns:
        M: Inertia matrix, shape (n_q, n_q).
        h: Bias term with qdd=0 in RNEA, shape (n_q,).

    Dynamics convention:
        M(q) * qdd + h(q, qd) = S^T * u_a
    where:
        M(q) is from CRBA,
        h(q,qd) = RNEA(q, qd, qdd=0).
    """

    # Normalize input arrays.
    q_vector = np.asarray(q, dtype=float).reshape(-1)
    qd_vector = np.asarray(qd, dtype=float).reshape(-1)
    model, data = ensure_model(n_m=q_vector.shape[0] - 3)

    # Validate model-consistent input dimensions.
    if q_vector.shape[0] != model.nq or qd_vector.shape[0] != model.nv:
        raise ValueError(f"Expected q shape ({model.nq},) and qd shape ({model.nv},)")

    # Compute M via CRBA and symmetrize numerically.
    inertia_matrix = pin.crba(model, data, q_vector)
    inertia_matrix = 0.5 * (inertia_matrix + inertia_matrix.T)

    # Compute bias h via RNEA at qdd = 0.
    bias_vector = pin.rnea(model, data, q_vector, qd_vector, np.zeros(model.nv))
    return np.asarray(inertia_matrix, dtype=float), np.asarray(bias_vector, dtype=float)
