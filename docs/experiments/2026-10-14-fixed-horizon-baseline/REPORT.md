# Fixed-horizon development comparator

24/24 trials completed with no runner errors: four scenarios, three robot families, two methods, seed 0.

All scenarios are development cases. Failures remain in the results. No initially colliding cases occurred.

| Method | Trials | Contact episodes | Final target completion | Collision-free completion | Watchdog-fault trials |
|---|---:|---:|---:|---:|---:|
| none | 12 | 11 | 4 | 1 | 3 |
| fixed-horizon | 12 | 9 | 1 | 1 | 3 |

## Interpretation

The fixed-horizon comparator avoided the narrow-passage contacts observed for KUKA and xArm, but neither arm completed that task under the comparator. Both methods completed the Panda narrow-passage case without contact. All three dynamic/occluded scenario types produced contact for both methods on all three families. The frozen-obstacle stop rule is therefore inadequate for these dynamic scenes; a stopped arm can still be struck by an approaching obstacle.

The four nominal target completions include three colliding trials. Completion alone must not be presented as safe success. The one collision-free completion for each method is the Panda narrow-passage case. All six occluded-crossing runs latched telemetry timeout; recovery of visual observations did not clear that execution fault.

This is a one-seed development diagnostic, not a tuned parameter sweep or statistical superiority result. In particular, fewer contacts accompanied reduced task progress. Do not use this outcome alone to claim that the proposed research method is justified or novel.

## Reproduction and provenance

Run `python tools/compare_research_baselines.py --output-dir experiments/baseline-reproduction` from the repository root. The directory must be empty. [Configuration and baseline semantics](../../research/BASELINE_SCENARIOS.md).

All 24 retained manifests have the same source snapshot hash and record a dirty working tree based on 7fe9837. They embed the resolved version-1 scenario, controller parameters, seed, defaults, dependencies and source-file hashes. Raw traces/events/predictions/decisions remain locally under ignored `experiments/baseline-release/`; the retained index and per-trial summaries record their trace hashes. Timing diagnostics depend on machine load.

Finite collision sampling, fixed obstacle observations, nominal velocity dynamics, local idealized watchdog stops and uncalibrated sensing uncertainty limit these results. This is an engineering comparator rather than a published baseline or formal safety controller.

Full regression: 519 passed, 7 optional skips. Focused scenario/baseline checks: 21 passed.

Validation details: [milestone status](../../status/2026-10-14-scenarios-and-baseline.md).
