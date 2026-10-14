# Claim and evidence register

Last updated: 2026-10-14. [Milestone evidence](../status/2026-10-14-research-foundation.md).

| Claim | Status | Evidence needed / available |
|---|---|---|
| Existing manipulator profiles execute in simulation | Established within recorded smoke scope | [xArm validation](../status/2026-10-14-xarm6-and-execution-monitor.md) |
| Independent plant/twin clients isolate physics state | Verified within test scope | `tests/test_plant_twin_research.py`: client isolation and non-mutating queries |
| Observations update the twin only after delivery | Verified within test scope | `tests/test_plant_twin_research.py`: delayed delivery and stale rejection |
| Identical scenario/seed replay produces equivalent traces | Verified in current environment | `tests/test_plant_twin_research.py`: exact repeated trace/event comparison |
| Candidate filter is novel | Unestablished | G0 review and technical distinction from closest methods |
| Candidate filter improves safety/progress | Untested | Tuned baselines, held-out trials, uncertainty analysis |
| Uncertainty estimates are calibrated | Untested | Disjoint calibration and coverage experiments |
| Collision avoidance has formal guarantees | Unestablished | Derivation with complete assumptions and independent review |
| System transfers to physical robots | Untested | Physical validation |

Software regression tests establish implementation behavior, not novelty,
statistical superiority, continuous collision freedom, or physical safety.
