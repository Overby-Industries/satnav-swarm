"""Absolute/relative position from inter-vehicle ranges (the "ranging" leg of
docs/approach-draft.md).

Given at least four anchors with known positions (a surface outpost, a
fleet member with a good fix, a beacon) and measured ranges to each, solve
for the receiver position with Gauss-Newton least squares. Also returns a
residual and a GDOP-style geometry figure so a caller can reject a fix that
is mathematically valid but geometrically poor.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

from .linalg import SingularMatrixError, Vec3, inverse_trace, norm, solve, sub


class NavError(ValueError):
    """The ranges/anchors cannot produce a usable fix."""


@dataclass(frozen=True)
class Fix:
    position: Vec3            # km, in the shared fleet reference frame
    rms_residual_km: float    # RMS of (measured - predicted) range
    gdop: float               # geometric dilution of precision; lower is better
    iterations: int
    converged: bool


def solve_position(
    anchors: Sequence[Vec3],
    ranges_km: Sequence[float],
    initial: Optional[Vec3] = None,
    max_iter: int = 50,
    tol_km: float = 1e-10,   # per km of position magnitude (relative)
) -> Fix:
    if len(anchors) != len(ranges_km):
        raise NavError("anchors and ranges must be the same length")
    if len(anchors) < 4:
        raise NavError("need at least 4 anchors for an unambiguous 3D fix")
    if any(r < 0 for r in ranges_km):
        raise NavError("ranges must be non-negative")

    n = len(anchors)
    if initial is None:
        initial = (
            sum(a[0] for a in anchors) / n,
            sum(a[1] for a in anchors) / n,
            sum(a[2] for a in anchors) / n,
        )
    p = list(initial)

    converged = False
    iterations = 0
    for iterations in range(1, max_iter + 1):
        # Jacobian rows are unit vectors from anchor to current estimate.
        h = []
        resid = []
        for a, r in zip(anchors, ranges_km):
            d = sub(p, a)
            dist = norm(d)
            if dist < 1e-12:
                # Estimate sits exactly on an anchor; nudge it off.
                p[0] += 1e-6
                d = sub(p, a)
                dist = norm(d)
            h.append([d[0] / dist, d[1] / dist, d[2] / dist])
            resid.append(r - dist)
        hth = [[sum(h[k][i] * h[k][j] for k in range(n)) for j in range(3)] for i in range(3)]
        htr = [sum(h[k][i] * resid[k] for k in range(n)) for i in range(3)]
        try:
            step = solve(hth, htr)
        except SingularMatrixError as exc:
            raise NavError("anchor geometry is degenerate (collinear or coplanar)") from exc
        for i in range(3):
            p[i] += step[i]
        # Relative to the magnitude of the position: an absolute tolerance is
        # unreachable in floating point once coordinates are thousands of km
        # (e.g. an Earth-centred frame), so a good fix would report converged=False.
        if norm(step) <= tol_km * max(1.0, norm(p)):
            converged = True
            break

    # Final residual and geometry at the solution.
    h = []
    sq = 0.0
    for a, r in zip(anchors, ranges_km):
        d = sub(p, a)
        dist = norm(d) or 1e-12
        h.append([d[0] / dist, d[1] / dist, d[2] / dist])
        sq += (r - dist) ** 2
    hth = [[sum(h[k][i] * h[k][j] for k in range(n)) for j in range(3)] for i in range(3)]
    try:
        gdop = inverse_trace(hth) ** 0.5
    except SingularMatrixError as exc:
        raise NavError("anchor geometry is degenerate (collinear or coplanar)") from exc

    return Fix((p[0], p[1], p[2]), (sq / n) ** 0.5, gdop, iterations, converged)
