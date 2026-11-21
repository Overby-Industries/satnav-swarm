"""Swarm-level navigation: propagate a fix outward from anchors, and screen
fleet members for close approaches.

Tiering follows starlifter-os/docs/ai-advisory-layer-draft.md: a close
approach that is imminent is Tier 0 (deterministic, no AI, no human in the
loop); one far enough out to deliberate over is a Tier 1 advisory item.
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import Dict, List, Mapping, Optional, Tuple

from .linalg import Vec3, add, dot, norm, scale, sub
from .trilateration import Fix, NavError, solve_position


def localize_swarm(
    anchors: Mapping[str, Vec3],
    ranges_km: Mapping[Tuple[str, str], float],
    max_gdop: float = 10.0,
) -> Tuple[Dict[str, Fix], List[str]]:
    """Fix every node that can be reached by ranging, bootstrapping outward.

    `ranges_km` maps unordered node-id pairs to a measured range. A node with
    ranges to >= 4 already-located nodes gets a fix and becomes an anchor for
    the next pass, so a swarm can be localized from a few seed anchors.
    Fixes whose GDOP exceeds `max_gdop` are rejected rather than propagated.
    Returns (fixes, unresolved node ids).
    """
    known: Dict[str, Vec3] = dict(anchors)
    fixes: Dict[str, Fix] = {}
    pair_range: Dict[frozenset, float] = {frozenset(k): v for k, v in ranges_km.items()}
    nodes = {n for pair in pair_range for n in pair} - set(anchors)

    progress = True
    while progress:
        progress = False
        for node in sorted(nodes - set(fixes)):
            a_pos, a_rng = [], []
            for other, pos in known.items():
                r = pair_range.get(frozenset((node, other)))
                if r is not None:
                    a_pos.append(pos)
                    a_rng.append(r)
            if len(a_pos) < 4:
                continue
            try:
                fix = solve_position(a_pos, a_rng)
            except NavError:
                continue
            if not fix.converged or fix.gdop > max_gdop:
                continue
            fixes[node] = fix
            known[node] = fix.position
            progress = True
    return fixes, sorted(nodes - set(fixes))


@dataclass(frozen=True)
class Conjunction:
    a: str
    b: str
    time_to_cpa_s: float
    miss_distance_km: float
    tier: str  # "tier0" (act deterministically now) or "tier1" (advisory)


def closest_approach(p1: Vec3, v1: Vec3, p2: Vec3, v2: Vec3) -> Tuple[float, float]:
    """(time_s >= 0, distance_km) of closest approach under straight-line motion."""
    dp, dv = sub(p2, p1), sub(v2, v1)
    vv = dot(dv, dv)
    t = 0.0 if vv == 0 else max(0.0, -dot(dp, dv) / vv)
    return t, norm(add(dp, scale(dv, t)))


def screen_conjunctions(
    states: Mapping[str, Tuple[Vec3, Vec3]],
    threshold_km: float,
    horizon_s: float,
    tier0_within_s: float,
) -> List[Conjunction]:
    """Pairwise close-approach screen. `states` maps id -> (position, velocity)."""
    out: List[Conjunction] = []
    for a, b in combinations(sorted(states), 2):
        t, d = closest_approach(*states[a], *states[b])
        if t <= horizon_s and d < threshold_km:
            out.append(Conjunction(a, b, t, d, "tier0" if t <= tier0_within_s else "tier1"))
    return sorted(out, key=lambda c: c.time_to_cpa_s)
