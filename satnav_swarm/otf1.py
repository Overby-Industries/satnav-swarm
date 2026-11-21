"""Turn a navigation solution into an OTF-1 `location` object.

OTF-1 (telemetry-protocol) is the schema for *reporting* position; this
module is the bridge from what satnav-swarm *computes* to that format. It
does not define a competing format.
"""
from __future__ import annotations

import math
from typing import Dict, Optional

from .linalg import Vec3, cross, norm

EARTH_RADIUS_KM = 6378.137
LEO_MAX_ALT_KM = 2_000.0
GEO_ALT_KM = 35_786.0
GEO_BAND_KM = 500.0
CISLUNAR_MAX_ALT_KM = 450_000.0

VALID_ZONES = {"LEO", "MEO", "GEO", "cislunar", "NEA", "belt", "surface"}


def classify_earth_zone(altitude_km: float) -> str:
    """Zone for a position given by altitude above Earth's equatorial radius."""
    if altitude_km < 0:
        raise ValueError("altitude below the reference radius; pass zone explicitly")
    if altitude_km <= LEO_MAX_ALT_KM:
        return "LEO"
    if abs(altitude_km - GEO_ALT_KM) <= GEO_BAND_KM:
        return "GEO"
    if altitude_km < GEO_ALT_KM:
        return "MEO"
    if altitude_km <= CISLUNAR_MAX_ALT_KM:
        return "cislunar"
    raise ValueError("beyond cislunar space; pass zone explicitly (NEA / belt)")


def inclination_deg(position_km: Vec3, velocity_km_s: Vec3) -> float:
    h = cross(position_km, velocity_km_s)
    hn = norm(h)
    if hn == 0:
        raise ValueError("zero angular momentum; inclination undefined")
    return math.degrees(math.acos(max(-1.0, min(1.0, h[2] / hn))))


def location_from_earth_state(
    position_km: Vec3,
    velocity_km_s: Optional[Vec3] = None,
    zone: Optional[str] = None,
) -> Dict[str, object]:
    """Earth-centred inertial state -> OTF-1 location object."""
    altitude = norm(position_km) - EARTH_RADIUS_KM
    loc: Dict[str, object] = {"zone": zone or classify_earth_zone(altitude)}
    if loc["zone"] not in VALID_ZONES:
        raise ValueError(f"unknown OTF-1 zone {loc['zone']!r}")
    loc["altitude_km"] = round(max(altitude, 0.0), 3)
    if velocity_km_s is not None:
        loc["inclination_deg"] = round(inclination_deg(position_km, velocity_km_s), 3)
    return loc
