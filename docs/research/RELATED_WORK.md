# Related work and gap audit

Reviewed 2026-10-14. This is an initial methods screening, not an exhaustive
systematic review. Full text was inspected for the five linked items below;
this does not constitute reproduction or verification of their proofs.
Unknown means not established by this review, not absent from the paper.

| Work and inspected sections | Relevant mechanism / evidence | Consequence for our contribution |
|---|---|---|
| [Betzer et al., Digital Twin Enabled Runtime Verification](https://arxiv.org/html/2412.09913v1), V–VI | Mobile-robot monitor specifications, MQTT integration, and actuation consistency monitoring | Digital-twin monitoring and timestamped traces are established ideas |
| [Busellato et al., UA-PCBF](https://arxiv.org/html/2508.20812v1), 3.3–3.4, 4, 6 | Forecast uncertainty modifies predictive separation constraints; reactive/predictive QP constraints have slack. Geometry models a hand and terminal robot region; physical experiments | Adaptive margins and uncertainty-aware predictive CBFs cannot be our novelty claim; whole-arm evaluation is a relevant distinction to investigate |
| [Compton et al., Predictive CBFs](https://arxiv.org/html/2412.04658v1), 2–4 | Full-order rollouts determine reduced-order robustness corrections; assumptions support theoretical results and hardware evaluation | Twin rollout and mismatch buffering alone are insufficient novelty |
| [Robust Safety-Critical Control for Input-Delayed System with Delay Estimation, ECC 2024](https://paperhost.org/proceedings/controls/ECC24/files/0872.pdf), III–V | Unknown bounded constant input delay, delay estimation, and prediction-error bounds enter robust barrier constraints | Delay compensation and delay-error robustification already exist; distinguish sensing age, queued commands, and variable transport delays |
| [Uncertainty-Aware Predictive Safety Filters for Probabilistic Neural Network Dynamics](https://arxiv.org/html/2604.26836v1), 4–5, appendices | Reachable-set approximation for uncertain learned dynamics, with treatment of predictive variance and linearization error | Uncertainty propagation or reachable tubes alone are not a new contribution; assumptions and approximation error need explicit treatment |

## Gate G0 decision

Status: **open; broad formulation overlaps prior work**. Proceed with testbed
development, but do not freeze an algorithm or claim an established research gap.

Candidate question: how should a whole-arm command filter allocate uncertainty
from capture age, commands already in flight, and empirically bounded tracking
residuals, without counting the same residual twice? Investigate whether this
yields a technical mechanism and a meaningful safety/progress advantage over
delay-aware robust filtering. This is our inference and proposed investigation,
not a claim about what all previous papers omit.

Before closing G0:

- Read delayed/sample-data CBF and robust predictive-filter references cited
  by the closest works; look for the exact combination above.
- Extract assumptions, state/input interfaces, hard versus soft constraints,
  backup policies, geometry coverage, and evaluation details into baseline specs.
- Check code/license availability and reproduce one simple published case.
- Resolve implementation-sensitive notation directly with original authors or
  supplementary material if needed; never silently repair and call it exact.
- Choose a published comparator with compatible information and actuation.
- Review novelty with a robotics/control researcher before main experiments.

The initial engineering comparator will be a fixed-horizon stop guard, not a
published-state-of-the-art baseline. Results against it alone cannot close G0.
