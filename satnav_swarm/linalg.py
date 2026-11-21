"""Tiny dependency-free linear algebra helpers (3-vectors and small dense solves).

Kept in pure Python on purpose: the nav core should run on a flight computer
or a CI box without numpy, and every system it solves here is at most 3x3.
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

Vec3 = Tuple[float, float, float]


def sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a: Sequence[float], k: float) -> Vec3:
    return (a[0] * k, a[1] * k, a[2] * k)


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Sequence[float]) -> float:
    return math.sqrt(dot(a, a))


class SingularMatrixError(ValueError):
    """Raised when a linear system has no unique solution."""


def solve(a: List[List[float]], b: List[float], eps: float = 1e-12) -> List[float]:
    """Solve a*x = b by Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < eps:
            raise SingularMatrixError("matrix is singular or ill-conditioned")
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(col + 1, n):
            f = m[r][col] / m[col][col]
            for c in range(col, n + 1):
                m[r][c] -= f * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


def inverse_trace(a: List[List[float]]) -> float:
    """Trace of a^-1, computed column by column."""
    n = len(a)
    total = 0.0
    for i in range(n):
        e = [0.0] * n
        e[i] = 1.0
        total += solve(a, e)[i]
    return total
