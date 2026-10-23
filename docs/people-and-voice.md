# Following people, and talking

## Finding and following people

`robot/person.py`. The motion computer already runs a person detector
(YOLOv5s on the RK3588's NPU, with tracking; it's what the phone app's
"follow" button uses). We decoded its UDP protocol from a capture of the app:
a 12-byte header `<code, json_length, 1>` then JSON, on port 43901.

- **`PersonDetector`** (a thread) switches detection on and receives
  ~27 target lists/s: `{"targets":[{"id", "bbox":[x1,y1,x2,y2]}]}` in
  1280×720 pixels. It locks onto the **largest box's track id** (usually the
  nearest person) and keeps following that id, re-locking only if it vanishes
  for over a second. Detections older than 0.7 s are ignored. With nobody in
  view the tracker sends nothing at all (measured 2026-10-06), so the detector
  also sends a state query (`0x21013305`) and takes its answer as proof the
  tracker is there; only no answer of either kind in 5 s is an error.
- We **don't use the built-in follow**: it drives at up to 1.0 m/s with no
  depth check. Driving goes through `bot.steer()` instead.
- **The controller** (`_controller`), every 50 ms:
  - turn rate = `−5.0 × (x − 0.5)`, clamped to 1.6 rad/s commanded, which
    is about 1.1 rad/s (64°/s) delivered (a proportional controller keeping
    the person centred in the image). It was 2.4 and 0.6 until 2026-10-06:
    0.6 commanded is 24°/s, far too slow to keep up with someone walking past;
  - but don't turn toward a side with something inside `TURN_SWEEP`;
  - walk forward while the person is within the middle half of the picture
    (`|x − 0.5| < 0.25`), turning as he goes. It was 0.12, which had him stop
    to re-aim at every sidestep; the job is keeping up, not pointing;
  - slow down linearly over the last 0.8 m before `stop_distance` (clearance
    is only sampled at 4 Hz, so full speed would overshoot), min 0.12 m/s;
  - no turning at all for an offset under 0.10: a small velocity is still a
    velocity, and any velocity keeps him stepping;
  - standing in front of the person in `follow()`, he stays put until they
    are 0.5 m beyond the stop distance or 0.25 off-centre, and once he turns
    he turns until they are within 0.10 again. Without those margins he
    shuffled on the spot, chasing depth and detection noise (2026-10-06);
  - if the person is lost: when they were last seen off-centre they walked
    out of the picture on that side, so turn that way at 1.0 rad/s commanded (half a
    turn in the 5 s below); lost near the middle, stand and wait;
  - stop if the person is lost for 5 s (`approach()`) or 15 s (the follows:
    close up the tracker often loses a person, seeing only legs, and a follow
    that ends because they stood near him for 5 s is no use), or the depth
    stream is lost.
- `approach()` ends at the stop distance; `follow()` holds position there,
  keeps turning to face the person, and walks again when they move away.
  `approach(near=fn)` calls `fn` ~1 m before arrival, which the tour uses to
  start writing its spoken line early so it can speak the moment it arrives.
  Both take `abort=fn`: a reason returned from it ends the move at the next
  cycle, which is how the panel's Cancel and E-STOP reach them.
- **`follow_nav()`** is a hybrid, for a room with furniture. It walks
  straight at the person with `follow()`'s own controller (quick: a decision
  every 50 ms, the speed asked for) while the depth camera shows the way to
  them free, and hands over to Nav2 goals while it does not. Nav2 alone was
  tried first and was slow: about 3 s to get going after each goal and
  0.3–0.4 m/s once moving (2026-10-06). "Free" means nothing more than 0.5 m
  nearer than the person, either straight ahead of him or within 0.3 m of
  the line to them. The costmap was tried for this and called a free line
  blocked at once and kept flipping near the person: it holds their own
  trail and their inflated outline. The way has to be blocked for 0.3 s
  before Nav2 takes over and free for 1 s before he walks straight again.
  What follows is the Nav2 part:
  `follow()` only ever steers straight at the person, `follow()` only ever steers straight at the person, so a
  chair on that line stops him for good. Every cycle it works out where the
  person is: bearing = `−(x − 0.5) × CAM_FOV` from the tracker's box; range
  from two sources. The box's height gives a rough one (`box_range()`: a
  1.87 m person (`PERSON_HEIGHT`, set to who he follows most), picture 73°
  top to bottom, so about 10 % off for someone 1.7 m, and nothing when the box is cut off because they are close). The
  depth scan gives an exact one, but of the nearest thing at that bearing,
  which among furniture is often a chair: one trace had a 1.9 m person read
  as 0.52 m, which ended the goal (2026-10-06). So the depth return nearest
  the box's estimate is used if one is within 35 % of it, else the box's
  estimate itself. It keeps a Nav2 goal
  (`goal_send()`, [goto(forward, heading_deg=None)](navigation.md#gotoforward-heading_degnone)) 0.6 m short of them on that line, replaced
  when they have moved 0.5 m (at most once a second) or every 2 s. Where they
  are is smoothed over a few fixes: single fixes jumped by 0.5 m and more,
  and each new goal makes Nav2 plan again, which was most of his hesitation
  (five goals in 1.4 s in one trace, 2026-10-06). Nav2 plans round what is in its
  costmap. Things to know:
  - beside him (more than 40° round, outside the depth camera's view) they
    get no goal: Nav2's costmap cannot see them either, and a goal there
    could walk him into them. A goal under way is dropped (it leads to
    where they were) and he turns to them;
  - a goal under way also survives a missed reading. The first version
    cancelled it on one, and stood (2026-10-06);
  - within 0.9 m (stop distance + Nav2's goal tolerance as it was then,
    0.25 m; it is 0.15 m since 2026-10-08, so Nav2 now brings him about
    0.1 m closer before he stands; not re-tested) he stands until they
    are 1.1 m away;
  - standing with no goal, he turns to them only once they are more than 30°
    round, at up to 1.2 rad/s commanded (0.84 delivered), and stops at 15°: enough to keep
    them in the depth view, not a correction at every arrival. He has no goal
    once Nav2 reports the last one finished (`goal_active()`); before that
    was checked he never turned after arriving, stood side-on and lost them;
  - a goal cell that is LETHAL or off the map is pulled back toward him in
    0.1 m steps to the first free one (NavFn would abort otherwise): the
    person's own trail stays in the costmap for a while after they move on;
  - if the person is lost while a goal is under way he finishes it, which
    brings him to where they were last seen, then gives up after 5 s;
  - something between him and the person at the same bearing reads as the
    person: he then comes to rest beside it until he sees them clear of it;
  - the speed argument is for the straight part; Nav2 sets its own pace;
  - the result's reason ends with how many goals were sent and the seconds
    spent in each state (`straight: walking / turning / standing / not seen`, `goal`, `with them`, `beside him`, `not seen`,
    `goal blocked`, `no range on them`), and every change of state is written to
    `/tmp/follow_nav.log`. Read that first when he stands about.

## Talking: the speaker and text-to-speech

`robot/talk.py`, two layers. Nothing is sent online except `play_url`.

### `Voice`: one sound at a time

The speaker is on the **motion computer**, so every sound is a pipeline
ending in `ssh ysc@192.168.1.120 aplay`:

```
text ─► Piper TTS ─► ffmpeg (to raw S16LE 48 kHz stereo, + 800 ms silence) ─► ssh ─► aplay ─► ES8388 ─► speaker
```

- `play('OKstandup')`: one of the robot's ~33 built-in clips; plays entirely
  on the robot.
- `say(text)`: Piper (neural TTS on the Jetson, offline), through `Speaker`
  below. No Piper, no speech: there is no fallback.
- `play_file(path)`, `play_url(youtube_url)` (via `yt-dlp`).

Details that were learned the hard way:

- `aplay` must target the hardware device `plughw:0,0`. The default device is
  a null sink, where `aplay` "succeeds" silently.
- The amplifier un-mutes when a stream starts and swallows the first
  moments, so 800 ms of silence is prepended (`LEAD_IN_MS`). A 1.1 s clip was
  completely inaudible without it.
- Raw samples, not WAV, go down the pipe, so there is no header to misread.
- Loudness (2026-09-18): the codec's PCM volume is set to max (192 = 0 dB,
  was 180 = −6 dB) each time the speaker opens, and speech gets a software
  gain of 1.6 with a limiter. Pushing the amp itself past 0 dB distorts on
  this small speaker.

What it needs: passwordless SSH from the Jetson to the motion computer; `ysc`
in the `audio` group there (done 2026-09-15); `ffmpeg` on the Jetson; Piper in
`~/piper` (the standalone aarch64 release, voices as `~/piper/<name>.onnx`);
for `play_url`, standalone `yt-dlp` and `deno` binaries in `~/.local/bin`
and internet. The robot has its own internet (eduroam via the Orin, see
"Driving across eduroam access points"). If it ever has none, run an HTTP
CONNECT proxy on the laptop, forward its port over SSH and set
`TALK_PROXY=http://localhost:3128`.

### `Speaker`: streaming, sentence by sentence

One long-running Piper process (model load is ~1.4 s) writes one WAV per
sentence; a pump thread feeds them into a single open `ffmpeg | ssh aplay`
stream. The first sentence plays while the next is still being synthesised.
It adds software gain with a limiter and sets the codec volume to max.

CLI: `python3 -m robot.talk say "text"`, `... play OKstandup`,
`... file <path>`, `... url <link>`, `... clips`, `... voices`.

Until 2026-10-23 a `Talker` here also chatted through Google Gemini, and
could describe a front camera frame ("Look and comment"). It was removed:
it sent camera frames and questions to Google in the USA, on a personal key,
for what was mostly a gimmick (PRIVACY.md). It is in git history.
