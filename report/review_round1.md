# Review of "Software Stack of the Lite3 Quadruped" (Round 1)

Mode: `full` (academic-paper-reviewer v1.11.1). Target: MAS518 lab report (Advanced Mechatronics
Project, project description "Learning to Walk and Think", A2026), which the user stated is a
guideline, not a checklist. Review date 2026-11-09.

**Provenance disclosure.** All five seats and the synthesis were run inline by one model (Claude
Opus 5.5), in sequence, in the same session that wrote the manuscript. The seats are role
separations, not independent reviewers: each could see the earlier seats' output and the author's
own drafting context. Same-family, same-author correlated errors are likely; treat agreement between
seats as weaker evidence than five real reviewers would give. The sprint-contract checkers, the
suggested-reference check and the reviewer claim check were not run (they need the plugin install
and per-seat artifacts). Status: `NOT_CALIBRATED`.

---

## Report 1: Journal-Fit Reviewer (target: MAS518 lab report)

**Recommendation:** Minor Revision. **Confidence:** 4

**Summary.** The report describes the software stack built for a Lite3 Pro: three computers, the
`Lite3` Python class, odometry fusion, map localization, Nav2 split across two computers, and a web
panel with speech. As a lab report for MAS518 it is well organised, readable and well illustrated,
and it explains design reasons, not only design. What a lab report for this course is missing is a
statement of which parts of the project description were taken on, and some evidence that the
system does what the report says. With those two additions it would serve both the examiners and
the next group taking over the robot.

### Strengths
**S1: Clear principle-driven framing.** The two principles in the introduction (turn silent failures
into fixes or errors; guarantee the stop) recur through every section and make the design choices
readable. Evidence Anchor: `text: §1 "every silent failure was turned into an automatic fix or a clear error, and the stop was guaranteed"`

**S2: Figures carry the explanation.** Six figures cover hardware, class structure, states, the
motion loop, odometry and navigation; each is referenced from the text. Evidence Anchor: `figure: Figures 1-6`

### Weaknesses
**W1: Scope against the project description is not stated.**
Problem: The project description lists four work packages (setup with simulation, multi-sensor
instrumentation, RL navigation, SLAM and navigation, for industrial inspection). The report never
says which were pursued, and frames the goal as walking around a campus.
Evidence Anchor: `absence: §1 Introduction — expected statement of project scope against the MAS518 work packages; checked §1, §8, abstract`
Why it matters: An examiner reads a lab report against the brief. Since the brief is a guideline,
not covering RL or simulation is fine, but it should be a stated choice.
Suggestion: Two or three sentences in §1: what the brief asked, which parts this work covered
(packages 1, 2 and 4), which it did not (RL, simulation, radar/thermal/GPS), and why.
Severity: Minor. Confidence: 4 (core: course reports).

**W2: No results section.**
Problem: The report is entirely descriptive; the only measurement is one loop-closure figure.
Evidence Anchor: `absence: whole manuscript — expected a results or testing section; checked §2-§8`
Why it matters: A lab report is expected to show that what was built works.
Suggestion: A short "Testing and Results" section (see Reviewer 1 for content).
Severity: Major. Confidence: 4.

**W3: The research question is not answered explicitly.**
Evidence Anchor: `text: §1 "How can an off-the-shelf quadruped be given a software stack that lets it navigate a building autonomously, safely and under remote supervision?"`
Problem: The conclusion restates what was built but does not return to the question's three terms
(autonomously, safely, under remote supervision).
Suggestion: Answer each term in the conclusion, with its limits.
Severity: Minor. Confidence: 4.

---

## Report 2: Peer Reviewer 1 (Methodology: experimental evaluation)

**Recommendation:** Major Revision. **Confidence:** 4

**Summary.** The report claims a working autonomous system but gives almost no measurements, and
the one quantitative odometry result is attributed to the wrong signal. Several strong claims
("guaranteed", "never made the pose jump", "could not outlive its operator") are stated as facts
with no test behind them in the text. The project's own documentation holds measurements that would
support most of these claims; they are simply not in the report. Adding a results table, correcting
the loop-closure attribution and stating test conditions would fix the main problems.

### Strengths
**S1: Specific numbers are given for design parameters.** Thresholds and rates are stated precisely
(50 ms loop, 30 s limit, 0.15 m and 0.2 rad gate, 1.5° tilt limit), so the design is checkable.
Evidence Anchor: `text: §4 "agreed with the legs' step within 0.15 m and 0.2 rad"`

### Weaknesses
**W1: The loop-closure result is attributed to the fused odometry, but it measured FAST-LIO2's pose.**
Problem: The report says the fused pose closed the loop to 0.05 m. The project's lidar documentation
records the 10.4 m loop as 0.05 m by `/lio_odom` (FAST-LIO2 alone) and 0.21 m by leg odometry, and
states that `/odom_fused` had not yet been walked at that test.
Evidence Anchor: `text: §4 "the fused pose returned to within 0.05 m of its start, against 0.21 m for leg odometry alone"`
Why it matters: It is the paper's only quantitative result, and it supports the fusion design that
it did not test.
Suggestion: Say "FAST-LIO2's pose" instead, or measure the fused pose on a walked loop and report that.
Severity: Major. Confidence: 5 (checked against docs/lidar.md, "Tested 2026-10-08").

**W2: Navigation performance is not quantified.**
Problem: No goal success rate, distance, speed, time or operator interventions are reported, though
records exist (for example, the first drive on the Orin's Nav2: lab door out and back, 11.7 s and
11.3 s, both goals reached, no recoveries; a later bridge walk: 3 of 3 goals reached).
Evidence Anchor: `absence: §6 Navigation — expected measured navigation outcomes; checked §6, §8`
Why it matters: The paper's central claim is autonomous navigation.
Suggestion: A table of the logged drives: route, length, goals reached, time, recoveries,
interventions, and known failure modes (the bridge glare stops).
Severity: Major. Confidence: 4.

**W3: Absolute claims without tests.**
Problem: "the stop was guaranteed", "never made the pose jump", "The robot could not outlive its
operator" are stated as properties, not as tested results.
Evidence Anchor: `text: §7 "The robot could not outlive its operator."`
Why it matters: Absolute claims need either a proof argument or a test; here neither is shown.
Suggestion: Report the tests that exist (the e-stop under Nav2 measured 0.000 m coast; the motion
loop is checked against a fake robot) and soften the rest to "is designed to".
Severity: Major. Confidence: 4.

**W4: Test conditions are missing for the numbers that are given.**
Evidence Anchor: `text: §7 "turned each sentence into audio in about 0.4 s"`
Problem: No n, date, conditions or what was timed (synthesis only; Say-to-sound measured 0.6-0.7 s).
Suggestion: State what each number measures and under which conditions.
Severity: Minor. Confidence: 4.

---

## Report 3: Peer Reviewer 2 (Domain: legged autonomy, fusion, Nav2)

**Recommendation:** Major Revision. **Confidence:** 4

**Summary.** The technical descriptions of FAST-LIO2, ICP localization, Smac and MPPI are accurate at
the level given, and splitting planning and control onto the Orin is well motivated by the
perception computer's load. The report gives no context, though: it does not say how comparable
quadruped stacks are built, why the vendor's own navigation was not used, or why a gated correction
was chosen over a standard filter. Known limitations recorded in the project documentation are left
out, which makes the system look more finished than it is.

### Strengths
**S1: The fusion design is explained with its failure behaviour.** The report says what happens when
FAST-LIO2 diverges or stalls, which is the part most descriptions omit.
Evidence Anchor: `text: §4 "a divergence, stall or restart of FAST-LIO2 never made the pose jump"`

**S2: Clear separation of responsibilities between computers.** Figure 6 and its text show which
side owns the map, the goal interface and the final say over the legs.
Evidence Anchor: `figure: Figure 6`

### Weaknesses
**W1: No background or state-of-the-art section.**
Problem: There is no comparison with existing legged autonomy stacks (vendor navigation, the CMU Go2
stack beyond its terrain analysis, Spot or ANYmal), nor with standard fusion (an EKF such as
`robot_localization`).
Evidence Anchor: `absence: between §1 and §2 — expected background on comparable systems and methods; checked §1-§3`
Why it matters: Without it the reader cannot tell which choices are standard and which are this
project's own.
Suggestion: A half-page background section (the course's bachelor-report style has one), and one
sentence each justifying the gated correction against an EKF and the own stack against the vendor's.
Severity: Major. Confidence: 4.

**W2: Known limitations are omitted.**
Problem: The documentation records that slow FAST-LIO2 drift is not caught by the gate, that
localization cannot detect sliding along a corridor, that the depth camera shows phantom obstacles
from glare on glossy floors and from metal-grid fences, and that a FAST-LIO2 run started while
settling is tilted and gives no fixes. None of these appear.
Evidence Anchor: `absence: §4-§6 — expected a limitations paragraph; checked §4, §5, §6, §8`
Suggestion: A short "Limitations" subsection or table.
Severity: Minor. Confidence: 5 (from the project's own docs).

**W3: Mapless mode is mentioned but not described.**
Evidence Anchor: `text: §6 "In mapless mode, everything ran on the perception computer in the odometry frame"`
Problem: Its controller (RPP) and costmap layers are not named, so the two modes cannot be compared.
Suggestion: One sentence naming the controller and why map mode moved to MPPI.
Severity: Minor. Confidence: 4.

---

## Report 4: Peer Reviewer 3 (Safety and HRI in public spaces)

**Recommendation:** Major Revision. **Confidence:** 4

**Summary.** The report takes safety seriously: the motion loop, the e-stop that kills Nav2, the
dead-man joystick and the mutual let-go between the computers are good practice. Its wording goes
beyond what a software-only stop can promise, and it says nothing about the people around a robot
that walks a campus with cameras. For a robot meant to be supervised from a desk, the failure path
of most interest (network lost while Nav2 drives) is described but its consequences are not.

### Strengths
**S1: Layered stop paths with explicit timeouts.** Dead-man at 0.3 s, goal cancel after 10 s
without a page, mutual let-go between computers, collision monitor last.
Evidence Anchor: `text: §6 "Each side let go when the other went quiet"`

### Weaknesses
**W1: "Guaranteed" stop overstates a software stop.**
Problem: Every stop except the handheld depends on software, processes and the network; during a
Wi-Fi gap the robot keeps navigating for up to 10 s under Nav2 and the collision monitor.
Evidence Anchor: `text: §1 "the stop was guaranteed, so that however a program ended, the last command sent to the legs was to stand still"`
Why it matters: In a public building a reader may take "guaranteed" as a safety rating.
Suggestion: Say what is guaranteed (the last command a program sends) and what is not (process
crashes of the bridge, power, network), and name the handheld as the only hardware stop early, not
only in the conclusion.
Severity: Major. Confidence: 4.

**W2: Bystanders and privacy are not addressed.**
Problem: The robot streams two cameras over the network while walking among students. The project
has a privacy document (firewalls, what is collected, how to run in public), but the report omits it.
Evidence Anchor: `absence: §7 — expected privacy and bystander handling; checked §2, §7, §8`
Suggestion: Two sentences on what is collected, who can see it, and the login and VPN restrictions.
Severity: Minor. Confidence: 4.

**W3: Supervision workload is asserted, not shown.**
Evidence Anchor: `text: §7 "let one operator supervise it remotely"`
Suggestion: Report latency (Tailscale direct ~6 ms on the campus network) and how often the operator
had to step in on the logged drives.
Severity: Minor. Confidence: 3.

---

## Report 5: Devil's Advocate

**Strongest counter-argument.** The report asks how an off-the-shelf quadruped can be made to
navigate a building "autonomously, safely and under remote supervision", and answers with an
architecture description. None of the three properties is demonstrated. Autonomy is partial: every
map is made by a person walking the robot with the handheld, and every navigation session starts
with the operator tapping the robot's position on the map. Safety rests on software stops and a
handheld that must be in someone's hand, which contradicts "supervised from a desk". Remote
supervision is shown as a list of panel features, not as an operator doing it. The conclusion's
central sentence, that the robot "could be sent around a campus from a desk", may well be true, but
nothing in the paper lets a reader check it. A sceptical examiner could read the report as excellent
system documentation that does not yet answer its own research question.

### Issues
- **CRITICAL: The core claim is unsupported in the text.** Dimension: argument/evidence. Location:
  §8, abstract. `text: §8 "turned a commercial quadruped into a robot that could be sent around a campus from a desk"`
  No section reports a campus run. Fix: the results section of Reviewer 1 W2.
- **MAJOR: "Supervised from a desk" vs "handheld kept at hand".** Location: abstract vs §7.
  `text: §7 "the handheld remote was kept at hand as the hardware stop"`. The two cannot both hold
  for one operator; say who holds the handheld during desk supervision.
- **MAJOR: Single measurement misattributed** (same as Reviewer 1 W1).
- **MINOR: Industrial-inspection brief vs campus framing.** The brief motivates offshore and plant
  inspection; the report never connects campus navigation back to it.

### Ignored alternatives
Using the vendor's own SLAM and navigation (the brief points to DEEP Robotics' resources) is not
discussed; neither is running everything on the Orin.

### Observations (non-defects)
The report is honest about the handheld being the only hardware stop, and the conclusion names a
hardware remote stop as next work.

---

## Editorial Decision

**Decision: Major Revision** (for a lab report: the revision is modest in effort, since most of the
missing evidence already exists in the project's logs and documentation).

**Consensus (4-5 seats):** the report needs a results or testing section (EIC W2, R1 W2-W3, R3 W3,
DA CRITICAL). **Majority (3 seats):** absolute safety and robustness wording needs softening or
evidence (R1 W3, R3 W1, DA MAJOR). **Single seat, high confidence:** the loop-closure number is
attributed to the wrong signal (R1 W1, echoed by DA); this is a factual error and must be fixed.

**DA CRITICAL adjudication:** validated. The conclusion's central claim has no supporting section.
It blocks Accept until a results section is added; it does not call the work itself into question.

**Disagreement:** The Journal-Fit Reviewer recommends Minor Revision, the others Major. The difference
is about the target: as a description of the system the report is nearly done; as a lab report that
answers its research question it is not. Major is chosen because of the validated CRITICAL.

### Revision Roadmap (in source order, not ranked)
1. Correct the loop-closure attribution (R1 W1).
2. Add a "Testing and Results" section: logged drives table (route, goals reached, time, recoveries,
   interventions), e-stop coast, turn accuracy, loop closure, speech latency, with conditions (R1
   W2-W4, EIC W2, DA CRITICAL, R3 W3).
3. Soften "guaranteed", "never" and "could not outlive" to what was tested or designed; say what the
   stop does not cover; name the handheld early (R1 W3, R3 W1).
4. State the project scope against the MAS518 brief in §1 (EIC W1, DA MINOR).
5. Add a short background section on comparable stacks and fusion methods, with one-line
   justifications (R2 W1, DA alternatives).
6. Add a limitations paragraph (R2 W2).
7. Answer the research question term by term in the conclusion (EIC W3).
8. Say who holds the handheld during desk supervision (DA MAJOR).
9. Name the mapless controller (R2 W3); add a privacy sentence (R3 W2).

Page budget: items 2, 5 and 6 add about a page. To stay at six pages, Figure 3 can be folded into
the text, and §7's panel feature list shortened.

---

## Attachment: Acronym Check (advisory, #849)
Script output (`check_acronyms.py`, on the PDF's text), unchanged in substance: not defined at first
use: ROS (14 uses), UDP (6), FAST-LIO2 (9), VPN (3), IMU, MID-360, RK3588, NX, CMU (2), WebRTC, GPU;
MPPI defined after first use (3). Reference-list venues (IEEE, IROS, ICRA, RSJ, MPC) need no
definition. Advisory only; not a reviewer finding.

## Attachment: Suggested-Reference Check (advisory, #1001)
The suggested-reference check did not run (reviewers suggested no specific works with identifiers).

## Attachment: Reviewer Claim Check (advisory, #1002)
The reviewer claim check did not run (needs the plugin install's script pipeline and a separate
judging subagent).
