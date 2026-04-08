"""Shared Python-side data structures for Route-B online control."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import numpy as np


@dataclass
class RouteBState:
    """Current generalized state returned by MuJoCo or another backend."""

    q: np.ndarray
    qd: np.ndarray
    time_s: float = 0.0


@dataclass
class TaskReference:
    """Task-space reference trajectory sample at the current control step."""

    x_des: np.ndarray
    xd_des: np.ndarray
    xdd_des: np.ndarray


@dataclass
class PinocchioTerms:
    """Pinocchio terms required by the online Route-B controller."""

    M: np.ndarray
    h: np.ndarray
    x_cur: np.ndarray
    xdot_cur: np.ndarray
    J_task: np.ndarray
    Jdot_qd: np.ndarray


@dataclass
class RouteBCommand:
    """Actuation-space command sent into the simulation backend."""

    u_a: np.ndarray
    qdd: Optional[np.ndarray] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class OnlineStepResult:
    """One online control-step bundle for diagnostics and logging."""

    state: RouteBState
    task: TaskReference
    terms: PinocchioTerms
    a_des: np.ndarray
    command: Optional[RouteBCommand] = None
    diagnostics: Dict[str, Any] = field(default_factory=dict)
