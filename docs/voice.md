# Talking: the speaker and text-to-speech

`robot/talk.py`, two layers. Nothing is sent online except `play_url`.

## `Voice`: one sound at a time

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

## `Speaker`: streaming, sentence by sentence

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

Person detection and following (`robot/person.py`: the motion computer's
built-in tracker, Walk up, Follow, the tour's `person` and `follow` steps)
were removed the same day for the same reason: they ran a detector over the
front camera's picture of people. Also in git history.
