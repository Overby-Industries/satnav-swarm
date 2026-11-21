"""Light-delay arithmetic. Ground tracking at belt distances is minutes to
hours stale, which is why it can't close a control loop (see
docs/approach-draft.md) and why Tier 0 hazard response never waits on it."""
from __future__ import annotations

C_KM_S = 299_792.458
AU_KM = 149_597_870.7


def one_way_delay_s(distance_km: float) -> float:
    if distance_km < 0:
        raise ValueError("distance must be non-negative")
    return distance_km / C_KM_S


def round_trip_delay_s(distance_km: float) -> float:
    return 2.0 * one_way_delay_s(distance_km)


def can_close_loop(distance_km: float, loop_period_s: float) -> bool:
    """True if a request/response over this link fits inside one loop period."""
    return round_trip_delay_s(distance_km) < loop_period_s
