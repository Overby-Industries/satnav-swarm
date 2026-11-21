# Satnav Swarm

**Overby Industries — autonomous satellite navigation for cislunar and asteroid belt operations.**

## Status: early -- ranging core is in

Built and tested (`pip install pytest && python -m pytest`), pure Python,
no dependencies:

- **`trilateration`** -- Gauss-Newton position fix from >= 4 anchor ranges,
  returning residual and GDOP so a geometrically poor fix can be rejected.
- **`swarm.localize_swarm`** -- bootstraps a whole swarm outward from a few
  seed anchors (each located node becomes an anchor), rejecting high-GDOP fixes.
- **`swarm.screen_conjunctions`** -- pairwise close-approach screen that
  labels each hit `tier0` (imminent: deterministic response, no AI) or
  `tier1` (far enough out to deliberate).
- **`light_delay`** -- can a request/response over this link close a loop?
- **`otf1`** -- converts an Earth-centred state into an OTF-1 `location`.

Not built: star-tracker/optical-landmark navigation, filtering over time,
and any OTF-1 representation of relative (formation) state -- OTF-1
`location` has only altitude/inclination/zone. Still the open trade study
in [`docs/approach-draft.md`](docs/approach-draft.md).

## Why This Is Its Own Repo

GPS doesn't reach cislunar space or the asteroid belt. A fleet operating
there -- Starlifters, miner pods, reclamation pods -- needs its own
relative/absolute navigation solution, shared across vehicle types rather
than reimplemented per repo.

## Relationship to Other Repos

- **[`starlifter-os`](https://github.com/Overby-Industries/starlifter-os)**
  is the fleet-coordination layer this would feed position/navigation data
  into and take navigation commands from.
- **[`telemetry-protocol`](https://github.com/Overby-Industries/telemetry-protocol)**'s
  OTF-1 `location` object (`altitude_km`, `inclination_deg`, `zone`) is the
  existing schema for reporting where a node is -- this repo is what
  *computes* that, not a competing format for reporting it.
- **[`power-and-propulsion`](https://github.com/Overby-Industries/power-and-propulsion)**'s
  `ShuttleFSM` has flight phases (`OrbitalCruise`, `ApproachAndLanding`)
  that would consume navigation solutions from this repo once both exist.

## Draft Scope

See [`docs/approach-draft.md`](docs/approach-draft.md). At minimum, a
credible design needs to address:

- **Absolute positioning** without GPS -- likely some combination of
  optical navigation (star trackers, known-landmark imaging) and
  inter-vehicle ranging.
- **Relative navigation** for swarm coordination -- formation-keeping and
  collision avoidance between fleet members at communication-delay-aware
  update rates.
- **Belt/cislunar-specific challenges**: sparse landmark availability,
  long light-delay to Earth-based tracking, dynamic environment (moving
  asteroids as both hazards and navigation references).

## Open Questions

- Centralized fleet-relative reference frame vs. each node navigating
  independently and reconciling -- same centralization question
  `starlifter-os` has for coordination, likely needs a consistent answer
  across both repos.
- What's the minimum viable sensor suite for a first demonstration vehicle,
  and how does that scale as the fleet grows?

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) and [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).
Orbital mechanics, GNC (guidance/navigation/control), and optical
navigation expertise are especially useful here.

## License

[Apache 2.0](LICENSE).
