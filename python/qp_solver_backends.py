"""QP solver backends for Route-B online control.

Supports:
- OSQP when installed
- SLSQP fallback via SciPy
"""

from __future__ import annotations

import importlib.util
from typing import Any, Dict

import numpy as np
from scipy.optimize import LinearConstraint, minimize
from scipy import sparse


def solve_qp(
    H: np.ndarray,
    g: np.ndarray,
    Aeq: np.ndarray,
    beq: np.ndarray,
    lb: np.ndarray,
    ub: np.ndarray,
    x0: np.ndarray,
    *,
    backend: str = "auto",
) -> Dict[str, Any]:
    """Solve a convex QP with equality constraints and box bounds."""

    requested = str(backend).lower()
    if requested == "auto":
        requested = "osqp" if _has_osqp() else "slsqp"

    if requested == "osqp":
        osqp_result = _solve_qp_osqp(H, g, Aeq, beq, lb, ub, x0)
        if osqp_result["success"]:
            return osqp_result
        if backend != "osqp":
            slsqp_result = _solve_qp_slsqp(H, g, Aeq, beq, lb, ub, x0)
            slsqp_result["fallback_from"] = "osqp"
            return slsqp_result
        return osqp_result

    return _solve_qp_slsqp(H, g, Aeq, beq, lb, ub, x0)


def _solve_qp_osqp(
    H: np.ndarray,
    g: np.ndarray,
    Aeq: np.ndarray,
    beq: np.ndarray,
    lb: np.ndarray,
    ub: np.ndarray,
    x0: np.ndarray,
) -> Dict[str, Any]:
    """Solve QP using OSQP if available."""

    if not _has_osqp():
        return {
            "success": False,
            "x": None,
            "status": "osqp_not_installed",
            "objective": float("nan"),
            "runtime_s": 0.0,
            "backend": "osqp",
        }

    import osqp  # type: ignore

    H = _regularize_hessian(H)
    g = np.asarray(g, dtype=float).reshape(-1)
    Aeq = np.asarray(Aeq, dtype=float)
    beq = np.asarray(beq, dtype=float).reshape(-1)
    lb = np.asarray(lb, dtype=float).reshape(-1)
    ub = np.asarray(ub, dtype=float).reshape(-1)
    x0 = np.asarray(x0, dtype=float).reshape(-1)
    n = int(g.size)

    A_blocks = []
    lower_blocks = []
    upper_blocks = []
    if Aeq.size > 0:
        A_blocks.append(sparse.csc_matrix(Aeq))
        lower_blocks.append(beq)
        upper_blocks.append(beq)
    A_blocks.append(sparse.eye(n, format="csc"))
    lower_blocks.append(lb)
    upper_blocks.append(ub)

    A = sparse.vstack(A_blocks, format="csc")
    l = np.concatenate(lower_blocks).astype(float)
    u = np.concatenate(upper_blocks).astype(float)
    P = sparse.csc_matrix(H)
    q = g.astype(float)

    solver = osqp.OSQP()
    try:
        solver.setup(
            P=P,
            q=q,
            A=A,
            l=l,
            u=u,
            verbose=False,
            polish=True,
            adaptive_rho=True,
            max_iter=10000,
            eps_abs=1e-6,
            eps_rel=1e-6,
            warm_start=True,
        )
        if x0.size == n and np.all(np.isfinite(x0)):
            solver.warm_start(x=x0)
        result = solver.solve()
    except Exception as exc:  # pragma: no cover - defensive
        return {
            "success": False,
            "x": None,
            "status": f"osqp_exception:{exc}",
            "objective": float("nan"),
            "runtime_s": 0.0,
            "backend": "osqp",
        }

    status_text = str(result.info.status).lower()
    success = "solved" in status_text
    return {
        "success": bool(success and result.x is not None),
        "x": None if result.x is None else np.asarray(result.x, dtype=float).reshape(-1),
        "status": str(result.info.status),
        "objective": float(result.info.obj_val) if np.isfinite(result.info.obj_val) else float("nan"),
        "runtime_s": float(getattr(result.info, "run_time", 0.0)),
        "backend": "osqp",
    }


def _solve_qp_slsqp(
    H: np.ndarray,
    g: np.ndarray,
    Aeq: np.ndarray,
    beq: np.ndarray,
    lb: np.ndarray,
    ub: np.ndarray,
    x0: np.ndarray,
) -> Dict[str, Any]:
    """Solve small convex QP with SciPy SLSQP."""

    H = _regularize_hessian(H)
    g = np.asarray(g, dtype=float).reshape(-1)
    Aeq = np.asarray(Aeq, dtype=float)
    beq = np.asarray(beq, dtype=float).reshape(-1)
    lb = np.asarray(lb, dtype=float).reshape(-1)
    ub = np.asarray(ub, dtype=float).reshape(-1)
    x0 = np.asarray(x0, dtype=float).reshape(-1)

    def objective(x: np.ndarray) -> float:
        return 0.5 * float(x @ H @ x) + float(g @ x)

    def gradient(x: np.ndarray) -> np.ndarray:
        return H @ x + g

    constraints = []
    if Aeq.size > 0:
        constraints.append(LinearConstraint(Aeq, beq, beq))
    result = minimize(
        objective,
        x0,
        method="SLSQP",
        jac=gradient,
        constraints=constraints,
        bounds=list(zip(lb, ub)),
        options={"maxiter": 300, "disp": False, "ftol": 1e-9},
    )
    return {
        "success": bool(result.success),
        "x": None if result.x is None else np.asarray(result.x, dtype=float).reshape(-1),
        "status": str(result.message),
        "objective": float(result.fun) if np.isfinite(result.fun) else float("nan"),
        "runtime_s": 0.0,
        "backend": "slsqp",
    }


def _regularize_hessian(H: np.ndarray) -> np.ndarray:
    H = np.asarray(H, dtype=float)
    return 0.5 * (H + H.T) + 1e-10 * np.eye(H.shape[0], dtype=float)


def _has_osqp() -> bool:
    return importlib.util.find_spec("osqp") is not None
