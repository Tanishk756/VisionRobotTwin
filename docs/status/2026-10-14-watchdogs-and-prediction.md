# Research execution deadlines and twin prediction

Implemented the next two tasks of the research roadmap in the simulation harness.
The interactive application's behavior is unchanged.

## Execution supervision

Velocity commands expire 150 ms after creation by default. Expired or reordered
obsolete deliveries cannot actuate the plant. Command starvation, missing initial
command deliveries, stale capture-time telemetry and missing startup telemetry
latch stop requests. Stops and reset cancel pending commands; motion requires
fresh telemetry and an explicit reset after a fault. Stop/reset failures remain
visible. The supervisor models a local stop outside the delayed transport queue.
It requests zero velocity; actual braking remains governed by plant physics.

## Prediction

The controller view exposes immutable command intentions and nominal arrival
bounds. The prediction API integrates piecewise-constant velocity intentions from
a delivered observation's capture time through now and a chosen horizon. It
includes command expiry, nominal watchdog deadlines and visible stop/reset
markers, and does not restart after a forecast deadline gap. FK samples preserve
the observed twin state and never query plant truth. Limit violations are flagged
rather than silently clipped. Prediction is unavailable after an unconfirmed halt.

Nominal transport timing is not the secretly sampled plant timing. Arrival bounds
are metadata rather than a propagated robust tube. The model assumes immediate
nominal velocity tracking, not identified motor dynamics or a braking model.
Telemetry-watchdog forecasts under future sensing loss remain outside this API.

## Validation and evidence

- Full regression run: 496 passed, 7 optional skips.
- Final focused research run: 44 passed, including the last forecast-latch fix
  and two additional prediction tests introduced after full-suite collection.
- Eight half-second smoke runs completed: six nominal runs without faults,
  startup dropout stopped at 0.300 s, and missing commands stopped at 0.150 s.
- [Experiment summaries and manifests](../experiments/2026-10-14-watchdogs-and-prediction/REPORT.md).
- [Current interfaces and reproduction guide](../research/TESTBED.md).

Next: versioned dynamic scenarios, an observation-only collision query interface,
fixed-horizon engineering comparator, uncertainty calibration, and the published
baseline/novelty review. The new predictive safety algorithm is not implemented
and G0 remains open.
