# Talking: the speaker and text-to-speech

`robot/talk.py`, two layers. Nothing is sent online except `play_url`.

## `Voice`: one sound at a time

The speaker is on the **motion computer**, so every sound is a pipeline
ending in `ssh ysc@192.168.1.120 aplay`:

```
text ─► Kokoro TTS (on the Orin) ─► ffmpeg (to raw S16LE 48 kHz stereo, + 800 ms silence) ─► ssh ─► aplay ─► ES8388 ─► speaker
```

- `play('OKstandup')`: one of the robot's ~33 built-in clips; plays entirely
  on the robot.
- `say(text)`: Kokoro (neural TTS on the Orin's GPU, offline, voice
  `bm_george`), through `RemoteSpeaker` below. Orin's voice down, no speech:
  there is no fallback (the user's choice, 2026-11-09). `say(text, voice=...)`
  still uses a Piper model on the Jetson.
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
and internet. The robot has its own internet (student-FoU via the Orin, see
"The robot's internet" in docs/panel.md). If it ever has none, run an HTTP
CONNECT proxy on the laptop, forward its port over SSH and set
`TALK_PROXY=http://localhost:3128`.

## Kokoro on the Orin (since 2026-11-09)

The user found Piper's voices hard to understand ("Hello" sounded wrong in
every one), so his voice is now Kokoro-82M, voice `bm_george` (British male,
the user's pick), on the Orin's GPU: ~0.4 s per sentence, 0.6-0.7 s from Say
to sound. `orin/bin/tts_server.py` keeps the model loaded and serves
sentences over TCP 8047 on the cable network only (192.168.1.5), so nobody on
campus Wi-Fi can make him talk; `env/kokoro-tts.service` runs it.

Installed on the Orin in `~/tts`: a venv (`python3 -m venv`, needed
`apt install python3-venv`) with `kokoro-onnx` 0.6.1 and `onnxruntime-gpu`
1.24 from the Jetson AI Lab index (`pypi.jetson-ai-lab.io/jp6/cu126`), and
the model and voices in `~/tts/models` (`kokoro-v1.0.onnx`,
`voices-v1.0.bin`). Trap: `pip install kokoro-onnx` pulls in the CPU
`onnxruntime`, which hides the GPU one; uninstall both and reinstall
`onnxruntime-gpu` with `--no-deps`. Check it with
`~/tts/venv/bin/python ~/bin/tts_server.py --check` ("Hello." in ~0.4 s, on
CUDAExecutionProvider). Supertonic was tried too: 1.1-2.3 s on the CPU.

The panel keeps one link open between Says. If it died (the Orin or the
service restarted) the Say is retried once on a fresh link.

## `Speaker`: streaming, sentence by sentence

One long-running Piper process (model load is ~1.4 s) writes one WAV per
sentence; a pump thread feeds them into a single open `ffmpeg | ssh aplay`
stream. The first sentence plays while the next is still being synthesised.
`RemoteSpeaker` is the same with the sentences made by `tts_server.py`.
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
