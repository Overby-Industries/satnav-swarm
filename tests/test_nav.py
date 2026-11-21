import math
import random

import pytest

from satnav_swarm import (
    NavError, can_close_loop, classify_earth_zone, closest_approach, localize_swarm,
    location_from_earth_state, one_way_delay_s, screen_conjunctions, solve_position,
)
from satnav_swarm.linalg import norm, sub

ANCHORS = [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (0.0, 100.0, 0.0), (0.0, 0.0, 100.0), (80.0, 90.0, 70.0)]


def ranges_to(p, anchors=ANCHORS):
    return [norm(sub(p, a)) for a in anchors]


def test_exact_ranges_recover_position():
    truth = (37.0, 52.0, 18.0)
    fix = solve_position(ANCHORS, ranges_to(truth))
    assert fix.converged
    assert norm(sub(fix.position, truth)) < 1e-6
    assert fix.rms_residual_km < 1e-6


def test_converges_at_earth_centred_magnitudes():
    truth = (7203.137, 0.0, 0.0)
    anchors = [(truth[0] + a[0], a[1], a[2]) for a in ANCHORS]
    fix = solve_position(anchors, ranges_to(truth, anchors))
    assert fix.converged and fix.iterations < 20
    assert norm(sub(fix.position, truth)) < 1e-6


def test_noisy_ranges_stay_close_and_report_residual():
    rng = random.Random(7)
    truth = (-40.0, 25.0, 60.0)
    noisy = [r + rng.gauss(0, 0.05) for r in ranges_to(truth)]
    fix = solve_position(ANCHORS, noisy)
    assert norm(sub(fix.position, truth)) < 0.5
    assert 0 < fix.rms_residual_km < 0.2


def test_too_few_anchors_rejected():
    with pytest.raises(NavError):
        solve_position(ANCHORS[:3], [1, 2, 3])


def test_coplanar_anchors_rejected():
    flat = [(0, 0, 0), (100, 0, 0), (0, 100, 0), (100, 100, 0)]
    with pytest.raises(NavError):
        solve_position(flat, [50, 60, 70, 80])


def test_swarm_bootstraps_outward_from_anchors():
    truth = {"A": (30.0, 30.0, 30.0), "B": (60.0, 20.0, 40.0)}
    anchors = {f"S{i}": a for i, a in enumerate(ANCHORS)}
    ranges = {}
    for i, a in enumerate(ANCHORS):
        ranges[(f"S{i}", "A")] = norm(sub(truth["A"], a))
    fixes, unresolved = localize_swarm(anchors, ranges)
    assert unresolved == [] and set(fixes) == {"A"}
    # B only sees A plus two anchors -> 3 known neighbours: not enough yet.
    ranges[("A", "B")] = norm(sub(truth["B"], truth["A"]))
    ranges[("S0", "B")] = norm(sub(truth["B"], ANCHORS[0]))
    ranges[("S1", "B")] = norm(sub(truth["B"], ANCHORS[1]))
    _, unresolved = localize_swarm(anchors, ranges)
    assert "B" in unresolved
    # Give B a fourth neighbour and it resolves via the bootstrapped A.
    ranges[("S2", "B")] = norm(sub(truth["B"], ANCHORS[2]))
    fixes, unresolved = localize_swarm(anchors, ranges)
    assert unresolved == []
    assert norm(sub(fixes["B"].position, truth["B"])) < 1e-5


def test_closest_approach_head_on():
    t, d = closest_approach((0, 0, 0), (1, 0, 0), (100, 1, 0), (-1, 0, 0))
    assert math.isclose(t, 50.0) and math.isclose(d, 1.0)


def test_diverging_objects_report_now():
    t, d = closest_approach((0, 0, 0), (-1, 0, 0), (10, 0, 0), (1, 0, 0))
    assert t == 0.0 and d == 10.0


def test_conjunction_tiering():
    states = {
        "A": ((0, 0, 0), (1, 0, 0)),
        "B": ((100, 0.5, 0), (-1, 0, 0)),       # cpa in 50 s -> tier0
        "C": ((0, 5000, 0), (0, -1, 0)),        # unrelated
        "D": ((100000, 0.5, 0), (-1, 0, 0)),    # far in time -> tier1
    }
    found = screen_conjunctions(states, threshold_km=2.0, horizon_s=1e6, tier0_within_s=600)
    tiers = {(c.a, c.b): c.tier for c in found}
    assert tiers[("A", "B")] == "tier0"
    assert tiers[("A", "D")] == "tier1"
    assert all(c.miss_distance_km < 2.0 for c in found)


def test_light_delay():
    assert math.isclose(one_way_delay_s(384_400), 1.282, rel_tol=1e-3)
    assert not can_close_loop(384_400, 1.0)
    assert can_close_loop(384_400, 5.0)


def test_zone_classification():
    assert classify_earth_zone(825) == "LEO"
    assert classify_earth_zone(20_200) == "MEO"
    assert classify_earth_zone(35_786) == "GEO"
    assert classify_earth_zone(380_000) == "cislunar"
    with pytest.raises(ValueError):
        classify_earth_zone(1_000_000)


def test_location_matches_otf1_example():
    r = 6378.137 + 825
    v = 7.4
    i = math.radians(51.6)
    loc = location_from_earth_state((r, 0, 0), (0, v * math.cos(i), v * math.sin(i)))
    assert loc["zone"] == "LEO"
    assert loc["altitude_km"] == 825.0
    assert loc["inclination_deg"] == 51.6
