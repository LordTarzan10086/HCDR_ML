"""QP solver backends for Route-B online control.

Supports:
- OSQP when installed
- SLSQP fallback via SciPy
"""

from __future__ import annotations

import importlib.util
import time
from typing import Any, Dict, Optional

import numpy as np
from scipy.optimize import LinearConstraint, minimize
from scipy import sparse


class OSQPReusableSolver:
    """Reusable OSQP wrapper for fixed-shape dense QP structures.

    The Route-B online controller solves many same-size QPs at each control
    step. Creating a fresh OSQP workspace dominates runtime, so this class
    keeps one workspace alive, updates numeric matrix/vector data, and
    preserves the last successful primal solution for warm-starting.
    """

    def __init__(
        self,
        H: np.ndarray,
        Aeq: np.ndarray,
        *,
        polish: bool = False,
        max_iter: int = 1000,
        eps_abs: float = 1e-4,
        eps_rel: float = 1e-4,
        adaptive_rho: bool = True,
    ) -> None:
        if not _has_osqp():
            raise RuntimeError("osqp_not_installed")

        import osqp  # type: ignore

        self._settings: Dict[str, Any] = {
            "verbose": False,
            "polishing": bool(polish),
            "adaptive_rho": bool(adaptive_rho),
            "max_iter": int(max_iter),
            "eps_abs": float(eps_abs),
            "eps_rel": float(eps_rel),
            "warm_starting": True,
        }
        self._solver = osqp.OSQP()
        self._n = int(np.asarray(H).shape[0])
        self._eq_rows = int(np.asarray(Aeq).shape[0])
        self._last_x: Optional[np.ndarray] = None
        self._last_success = False

        P = _dense_upper_csc(_regularize_hessian(H))
        A = self._build_constraint_matrix(Aeq)
        zero_q = np.zeros(self._n, dtype=float)
        lower = -np.inf * np.ones(A.shape[0], dtype=float)
        upper = np.inf * np.ones(A.shape[0], dtype=float)
        self._P_indices = P.indices.copy()
        self._P_indptr = P.indptr.copy()
        self._A_indices = A.indices.copy()
        self._A_indptr = A.indptr.copy()
        self._solver.setup(P=P, q=zero_q, A=A, l=lower, u=upper, **self._settings)

    @property
    def last_x(self) -> Optional[np.ndarray]:
        """Last successful primal solution, used by the controller as x0."""

        if self._last_x is None:
            return None
        return self._last_x.copy()

    def update_settings(self, **kwargs: Any) -> None:
        """Update OSQP runtime settings without rebuilding the workspace."""

        accepted = {"polish", "polishing", "max_iter", "eps_abs", "eps_rel", "adaptive_rho", "verbose"}
        updates = {}
        for key, value in kwargs.items():
            if key not in accepted or value is None:
                continue
            settings_key = "polishing" if key == "polish" else key
            updates[settings_key] = value
        if not updates:
            return
        self._settings.update(updates)
        self._solver.update_settings(**updates)

    def can_update(self, H: np.ndarray, Aeq: np.ndarray) -> bool:
        """Return true when the new problem can reuse this workspace."""

        H = np.asarray(H, dtype=float)
        Aeq = np.asarray(Aeq, dtype=float)
        return H.shape == (self._n, self._n) and Aeq.shape[0] == self._eq_rows and Aeq.shape[1] == self._n

    def update_problem_matrices(self, H: np.ndarray, Aeq: np.ndarray) -> None:
        """Update Hessian and equality-constraint numeric values.

        Dense explicit sparsity is used for the Hessian upper triangle and the
        equality block, so the OSQP sparsity pattern remains stable across
        changing dynamics/Jacobian values.
        """

        if not self.can_update(H, Aeq):
            raise ValueError("osqp_reusable_shape_mismatch")
        P = _dense_upper_csc(_regularize_hessian(H))
        A = self._build_constraint_matrix(Aeq)
        if (
            not np.array_equal(P.indices, self._P_indices)
            or not np.array_equal(P.indptr, self._P_indptr)
            or not np.array_equal(A.indices, self._A_indices)
            or not np.array_equal(A.indptr, self._A_indptr)
        ):
            raise ValueError("osqp_reusable_sparsity_mismatch")
        self._solver.update(Px=P.data, Ax=A.data)

    def solve(
        self,
        g: np.ndarray,
        beq: np.ndarray,
        lb: np.ndarray,
        ub: np.ndarray,
        x0: Optional[np.ndarray] = None,
    ) -> Dict[str, Any]:
        """Update vector data, warm-start, and solve the reusable OSQP QP."""

        vector_update_start = time.perf_counter()
        q = np.asarray(g, dtype=float).reshape(-1)
        beq_vec = np.asarray(beq, dtype=float).reshape(-1)
        lb_vec = np.asarray(lb, dtype=float).reshape(-1)
        ub_vec = np.asarray(ub, dtype=float).reshape(-1)
        if q.size != self._n or lb_vec.size != self._n or ub_vec.size != self._n or beq_vec.size != self._eq_rows:
            self._clear_warm_start()
            return {
                "success": False,
                "x": None,
                "status": "osqp_reusable_vector_shape_mismatch",
                "objective": float("nan"),
                "runtime_s": 0.0,
                "backend": "osqp_reusable",
                "profile": {"osqp_vector_update_ms": 0.0, "osqp_solve_ms": 0.0},
            }

        lower = np.concatenate([beq_vec, lb_vec]).astype(float)
        upper = np.concatenate([beq_vec, ub_vec]).astype(float)
        try:
            self._solver.update(q=q.astype(float), l=lower, u=upper)
            vector_update_ms = (time.perf_counter() - vector_update_start) * 1000.0
            warm_start_start = time.perf_counter()
            warm_start = None if x0 is None else np.asarray(x0, dtype=float).reshape(-1)
            if warm_start is not None and warm_start.size == self._n and np.all(np.isfinite(warm_start)):
                self._solver.warm_start(x=warm_start)
            warm_start_ms = (time.perf_counter() - warm_start_start) * 1000.0
            solve_start = time.perf_counter()
            result = self._solver.solve()
            solve_ms = (time.perf_counter() - solve_start) * 1000.0
        except Exception as exc:  # pragma: no cover - defensive
            self._clear_warm_start()
            return {
                "success": False,
                "x": None,
                "status": f"osqp_reusable_exception:{exc}",
                "objective": float("nan"),
                "runtime_s": 0.0,
                "backend": "osqp_reusable",
                "profile": {
                    "osqp_vector_update_ms": (time.perf_counter() - vector_update_start) * 1000.0,
                    "osqp_warm_start_ms": 0.0,
                    "osqp_solve_ms": 0.0,
                },
            }

        status_text = str(result.info.status).lower()
        success = "solved" in status_text and result.x is not None
        if success:
            self._last_success = True
            self._last_x = np.asarray(result.x, dtype=float).reshape(-1).copy()
        else:
            self._clear_warm_start()
        return {
            "success": bool(success),
            "x": None if result.x is None else np.asarray(result.x, dtype=float).reshape(-1),
            "status": str(result.info.status),
            "objective": float(result.info.obj_val) if np.isfinite(result.info.obj_val) else float("nan"),
            "runtime_s": float(getattr(result.info, "run_time", 0.0)),
            "backend": "osqp_reusable",
            "profile": {
                "osqp_vector_update_ms": float(vector_update_ms),
                "osqp_warm_start_ms": float(warm_start_ms),
                "osqp_solve_ms": float(solve_ms),
            },
        }

    def _build_constraint_matrix(self, Aeq: np.ndarray) -> sparse.csc_matrix:
        Aeq_full = _dense_csc_full_pattern(np.asarray(Aeq, dtype=float), shape=(self._eq_rows, self._n))
        return sparse.vstack([Aeq_full, sparse.eye(self._n, format="csc")], format="csc")

    def _clear_warm_start(self) -> None:
        self._last_success = False
        self._last_x = None


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


def _dense_upper_csc(matrix: np.ndarray) -> sparse.csc_matrix:
    """Store the full upper triangle, including explicit zeros, as CSC."""

    matrix = np.asarray(matrix, dtype=float)
    if matrix.ndim != 2 or matrix.shape[0] != matrix.shape[1]:
        raise ValueError(f"Expected square Hessian, got shape {matrix.shape}.")
    rows = []
    cols = []
    data = []
    n = matrix.shape[0]
    for col in range(n):
        for row in range(col + 1):
            rows.append(row)
            cols.append(col)
            data.append(float(matrix[row, col]))
    return sparse.csc_matrix((data, (rows, cols)), shape=(n, n))


def _dense_csc_full_pattern(matrix: np.ndarray, *, shape: tuple[int, int]) -> sparse.csc_matrix:
    """Store every matrix entry as explicit CSC data for reusable updates."""

    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != shape:
        raise ValueError(f"Expected matrix shape {shape}, got {matrix.shape}.")
    rows = []
    cols = []
    data = []
    row_count, col_count = shape
    for col in range(col_count):
        for row in range(row_count):
            rows.append(row)
            cols.append(col)
            data.append(float(matrix[row, col]))
    return sparse.csc_matrix((data, (rows, cols)), shape=shape)


def has_osqp() -> bool:
    """Public availability probe used by the online controller."""

    return _has_osqp()


def _has_osqp() -> bool:
    return importlib.util.find_spec("osqp") is not None
