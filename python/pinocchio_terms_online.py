"""Online Pinocchio term collection for Route-B control."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable

import numpy as np

from common_types import PinocchioTerms

_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_DIR = _REPO_ROOT / "src"
if str(_SRC_DIR) not in __import__("sys").path:
    __import__("sys").path.insert(0, str(_SRC_DIR))

from pin_terms import get_M_h, get_pose_jacobian_terms  # noqa: E402


def compute_pinocchio_terms(
    q: Iterable[float],
    qd: Iterable[float],
    model_kwargs: Dict[str, Any],
) -> PinocchioTerms:
    """Return the full online Route-B Pinocchio bundle.

    Parameters
    ----------
    q, qd:
        Current generalized state, shaped as [x, y, psi, q_m...].
    model_kwargs:
        Keyword arguments forwarded to src/pin_terms.py model builders.
    """

    q_vec = np.asarray(q, dtype=float).reshape(-1)
    qd_vec = np.asarray(qd, dtype=float).reshape(-1)
    M, h = get_M_h(q_vec, qd_vec, **model_kwargs)
    x_cur, xdot_cur, J_task, Jdot_qd = get_pose_jacobian_terms(q_vec, qd_vec, **model_kwargs)
    return PinocchioTerms(
        M=np.asarray(M, dtype=float),
        h=np.asarray(h, dtype=float).reshape(-1),
        x_cur=np.asarray(x_cur, dtype=float).reshape(-1),
        xdot_cur=np.asarray(xdot_cur, dtype=float).reshape(-1),
        J_task=np.asarray(J_task, dtype=float),
        Jdot_qd=np.asarray(Jdot_qd, dtype=float).reshape(-1),
    )
