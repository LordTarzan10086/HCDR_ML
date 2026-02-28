"""Pinocchio dynamics terms for Route-B: M via CRBA, h via RNEA(qdd=0)."""

from __future__ import annotations

from typing import Iterable, Optional, Tuple

import numpy as np
import pinocchio as pin

from pin_model_build import build_planar_root_arm_model

_MODEL = None
_DATA = None


def ensure_model(n_m: int = 6, link_lengths: Optional[Iterable[float]] = None):
    global _MODEL, _DATA
    if _MODEL is None:
        _MODEL = build_planar_root_arm_model(n_m=n_m, link_lengths=link_lengths, micro_g=True)
        _DATA = _MODEL.createData()
    return _MODEL, _DATA


def get_M_h(q: Iterable[float], qd: Iterable[float]) -> Tuple[np.ndarray, np.ndarray]:
    q = np.asarray(q, dtype=float).reshape(-1)
    qd = np.asarray(qd, dtype=float).reshape(-1)
    model, data = ensure_model(n_m=q.shape[0] - 3)

    if q.shape[0] != model.nq or qd.shape[0] != model.nv:
        raise ValueError(f"Expected q shape ({model.nq},) and qd shape ({model.nv},)")

    M = pin.crba(model, data, q)
    M = 0.5 * (M + M.T)
    h = pin.rnea(model, data, q, qd, np.zeros(model.nv))
    return np.asarray(M, dtype=float), np.asarray(h, dtype=float)
