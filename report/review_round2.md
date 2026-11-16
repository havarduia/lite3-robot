# Review of "Software Stack of the Lite3 Quadruped" (Round 2, re-review)

Mode: `re-review` (academic-paper-reviewer v1.11.1). Target unchanged: MAS518 lab report. Round-1 cards
reused (yardstick continuity); `field_analyst_agent` not re-run. Date 2026-11-09.

**Provenance disclosure.** Run inline by one model (Claude Opus 5.5), the same session that wrote and
revised the manuscript. The three-gate contract could not hold: the verifier had already seen the revision
before Phase 1 (it made it), so Phase 1 was not revision-blind and Phase 2A was not persuasion-blind. There
is no separate Response to Reviewers; the author's claims are the revision summary given in chat. The JSON
sidecars, input manifest and `check_re_review_synthesis.py` were not produced or run. Treat verdicts as a
careful self-check, not an independent verification. Status: `NOT_CALIBRATED`.

Round-1 roadmap classes (Round 1 did not label them; assigned here from its Editorial Decision):
must_fix = items 1-3 (single-seat factual error, consensus/DA CRITICAL, majority); should_fix = 4-8;
consider = 9.

## Phase 1: criteria (from Round-1 artifacts)

| # | Class | Fully addressed when |
|---|-------|----------------------|
| 1 | must_fix | The 0.05 m loop figure is attributed to the signal it was measured on |
| 2 | must_fix | A results section reports the logged drives (route, goals, time, recoveries, interventions), e-stop, turns, loop, speech, with conditions; the conclusion's claim rests on it |
| 3 | must_fix | No absolute safety wording beyond what was designed or tested; what the stop does not cover is said; handheld named early |
| 4 | should_fix | §1 places the work against the MAS518 brief |
| 5 | should_fix | Short background on comparable stacks and fusion, with a reason for the choices |
| 6 | should_fix | A limitations paragraph |
| 7 | should_fix | The conclusion answers the research question term by term |
| 8 | should_fix | Says who holds the handheld during supervision |
| 9 | consider | Mapless controller named; a privacy sentence |

## Phase 2A: evidence verdicts

| # | Verdict | Evidence (revised manuscript) | Change |
|---|---------|-------------------------------|--------|
| 1 | FULLY_ADDRESSED | Table 1 "Odometry, walked loop": "FAST-LIO2 ended 0.05 m from the start, leg odometry 0.21 m"; matches docs/lidar.md:153 (`/lio_odom`, `/leg_odom2`). §5 no longer claims a fused loop closure | Attribution moved from `/odom_fused` to FAST-LIO2 |
| 2 | PARTIALLY_ADDRESSED | §9 + Table 1: ten rows, all traced to logs or docs (see Fact check). Conditions given once ("people moving around it, a person carrying the handheld") | Section added. **Residual (should_fix):** interventions and recoveries are reported for the door run only. The bridge row says "3 of 3 goals" but not that the outward leg stalled four times on glare and the return used backup and spin recoveries (Orin log 2026-11-04), so the table reads cleaner than the run was. The limitations paragraph partly covers it |
| 3 | PARTIALLY_ADDRESSED | "guaranteed", "could not outlive" gone; §1 ends "the handheld remote remained the robot's only hardware stop"; §8 "These stops depend on the software and the network" | Softened. **Residual (should_fix):** §1 still says "however a program ended, the last command it sent to the legs was to stand still". A `finally` block does not run on SIGKILL, an interpreter crash or a lost Ethernet link, so this is still absolute. Fix: "whenever a program ended normally, by an exception or by Ctrl-C" |
| 4 | FULLY_ADDRESSED | §1 third paragraph: the MAS518 parts covered and the parts left out, pointing to §10 | Added |
| 5 | FULLY_ADDRESSED | §2 Background: ANYmal, CMU Go2, FAST-LIO2, ICP, Nav2/RPP/MPPI, EKF, and why a gate was used | Added (but see new issue N1) |
| 6 | FULLY_ADDRESSED | §9 last paragraph: bridge glare, corridor sliding, fences, FAST-LIO2 initialized while settling | Added |
| 7 | FULLY_ADDRESSED | §11 answers *Autonomously*, *Safely* and *Under remote supervision* in turn | Rewritten |
| 8 | FULLY_ADDRESSED | Abstract, §8, §9, §11: a person with the handheld nearby during tests | Added |
| 9 | PARTIALLY_ADDRESSED (consider, no effect on the decision) | §7 names RPP for mapless mode. No privacy sentence; §3 only says video streams ran while a page was open. The project has PRIVACY.md and a sign (docs/privacy-sign.html) that could be cited in one line | Half done |

### New issues (frozen at EVIDENCE-COMMITTED)

| ID | Attribution | Severity | Finding |
|----|-------------|----------|---------|
| N1 | regression (new text) | minor | §2: "Such a filter averages its inputs, so a lidar odometry that diverges also pulls the estimate away." `robot_localization` has Mahalanobis-distance rejection of outlier measurements, so it can gate too. A fairer reason: the gate compares *steps* against the legs at matched time stamps and holds the correction on a disagreement, which a per-measurement threshold tuned for noise does not do by default. Reword to "unless its outlier rejection is tuned, ..." or give the step-comparison reason |
| N2 | previously_missed | minor | §6: "refusing fixes while FAST-LIO2's frame was more than 1.5° off level". In `robot/cloud_locate.py:169-172` the tilt check applies only before the robot is placed (once it walks, the reading follows its gait). Say "refusing to place the robot while ..." |
| N3 | previously_missed | minor | §1 lists "robot configuration and teleoperation" as covered, but the brief's first part also names kinematics; the report says nothing on it. Either a clause in §10 or "configuration and teleoperation (not kinematics)" |

### Fact check of Table 1 (all traced, no mismatch)

| Row | Source |
|-----|--------|
| Loop 10.4 m / 0.05 / 0.21 m | docs/lidar.md:153, docs/background.md:55 (time 92 s from the same log, not re-checked here) |
| Leg scale 1.712 / 1.960, ×1.15 | docs/environment.md:66 |
| Turns 29.3-30.3°, 0.17 s | docs/api.md:139-142, docs/environment.md:67 |
| E-stop 0.000 m | docs/api.md:187, docs/environment.md:82 (`demos/estop_nav.py`) |
| Door 11.7 / 11.3 s, median 0.48 m/s | orin_nav1, 2026-11-02 |
| MPPI ~10 s vs RPP 14-15 s, 0.16-0.18 m/s | mppi_drive4 vs shadow3, 2026-11-02 (MPPI on the perception computer then, before it moved to the Orin; the row does not say which) |
| Lab and hall 7/7 | Orin log big_walk_to_arcade, 2026-11-04 |
| Arcade 3/3, 200 s | Orin log 2026-11-04 evening |
| Speech 0.4 s, 0.6-0.7 s | docs/voice.md:52 |
| VPN 6 ms | docs/panel.md:89 |

Design constants in the text checked against code: battery 20 %, tilt 30°, 30 s ceiling
(`robot/lite3.py`), dead-man 0.3 s and 10 s page-gone cancel (`robot/hmi.py`), clamp −0.2..0.5 m/s
(`robot/shadow_link.py`), 1.5° tilt (`robot/cloud_locate.py`, see N2).

## Phase 2B: claim matching
No Response to Reviewers. The author's summary in chat matches the evidence above except that it
counts items 2 and 3 as done; both have the residuals given. No verdict adjustments.

## Decision derivation
- must_fix: 1 FULLY, 2 and 3 PARTIALLY with should_fix residuals → B5
- should_fix_addressed_rate: 5/5 = 100 %
- One regression-attributed minor new issue (N1) → B5

**Decision: Minor Revision.** (Round 1: Major Revision.)

### Remaining work (about 15 minutes, no page cost)
1. §1: limit "however a program ended" to normal ends, exceptions and Ctrl-C (item 3).
2. Table 1: add the bridge stalls and recoveries to the arcade row, and say MPPI ran on the
   perception computer in the comparison (item 2).
3. §2: fix the EKF sentence (N1).
4. §6: "refusing to place the robot while ..." (N2).
5. Optional: one privacy sentence (item 9) and a kinematics clause (N3).

## Attachment: Acronym Check (advisory, #849)
`check_acronyms.py` on the PDF text, IEEE/IROS/ICRA/RSJ/MPC allowed. Flags DEEP, MAS518, ANYmal, NX, RK3588,
MID (product and course names, no definition needed), ROS and FAST-LIO2 (defined with a space or in
another form; the script misses it), VPN "after first use" (the first use is inside Figure 1), GPS
and RViz (not defined; GPS is common, RViz could get "the ROS visualizer"). Advisory only.
