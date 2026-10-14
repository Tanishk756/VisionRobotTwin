# Claim and evidence register

Last updated: 2026-10-14. [Milestone evidence](../status/2026-10-14-research-foundation.md).

| Claim | Status | Evidence needed / available |
|---|---|---|
| Existing manipulator profiles execute in simulation | Established within recorded smoke scope | [xArm validation](../status/2026-10-14-xarm6-and-execution-monitor.md) |
| Independent plant/twin clients isolate physics state | Verified within test scope | `tests/test_plant_twin_research.py`: client isolation and non-mutating queries |
| Observations update the twin only after delivery | Verified within test scope | `tests/test_plant_twin_research.py`: delayed delivery and stale rejection |
| Identical scenario/seed replay produces equivalent traces | Verified in current environment | `tests/test_plant_twin_research.py`: exact repeated trace/event comparison |
| Deadline-bearing commands and watchdog faults prevent obsolete dispatch | Verified within simulation test scope | `tests/test_research_watchdog_prediction.py`: expiry, starvation, missing startup telemetry, cancellation and reset |
| Nominal prediction uses delivered observations and intention history without plant truth | Verified within test scope | `tests/test_research_watchdog_prediction.py`: analytical integration, changed hidden state, FK preservation and nominal timing |
| Versioned scenarios and observation-only whole-arm queries execute reproducibly | Verified within development/test scope | `tests/test_research_scenarios_baseline.py` and [24-trial report](../experiments/2026-10-14-fixed-horizon-baseline/REPORT.md) |
| Fixed-horizon stop comparator has tested HOLD recovery and latched faults | Verified within implementation test scope | `tests/test_research_scenarios_baseline.py`; this does not establish collision avoidance |
| Deterministic motion-aware comparator uses delivered means without covariance constraints | Verified within implementation/development scope | `tests/test_motion_aware_baseline.py`; [36-trial report](../experiments/2026-10-14-motion-aware-baseline/REPORT.md) shows no aggregate improvement over the frozen baseline |
| Candidate filter is novel | Unestablished | G0 review and technical distinction from closest methods |
| Candidate filter improves safety/progress | Untested | Tuned baselines, held-out trials, uncertainty analysis |
| Uncertainty estimates are calibrated | Not established; undercoverage measured under motion/delay shift | [First diagnostic](UNCERTAINTY_CALIBRATION.md): disjoint fit/validation cases; nominal 95% coverage below target |
| Collision avoidance has formal guarantees | Unestablished | Derivation with complete assumptions and independent review |
| System transfers to physical robots | Untested | Physical validation |

Software regression tests establish implementation behavior, not novelty,
statistical superiority, continuous collision freedom, or physical safety.
Nominal prediction is kinematic and does not establish a dynamics or braking
model. Arrival bounds are recorded but not propagated as a safety tube.
