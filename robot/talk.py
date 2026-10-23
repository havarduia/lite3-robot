#!/usr/bin/env python3
"""talk - everything the robot says, out of its own speaker.

The speaker is on the MOTION computer, so playing a sound is ssh + aplay.
Voice plays one sound: a built-in clip, a file, a URL, or text through Piper
(offline text-to-speech, on this computer). Speaker is the streaming Piper
behind Voice.say. Nothing here goes to the internet except play_url. How it
works and what it needs installed: docs/voice.md.

    from robot.talk import Voice
    Voice().play('OKstandup')                  # one of the robot's ~33 clips
    Voice().say('watch your step')             # text-to-speech

From the shell:

    python3 -m robot.talk say "hello there"    # speak exactly this text
    python3 -m robot.talk play OKstandup       # a built-in clip
    python3 -m robot.talk file ~/alarm.wav     # any local audio
    python3 -m robot.talk url <youtube url>    # a video's audio
    python3 -m robot.talk clips | voices       # list the clips / Piper voices
  options: --voice PIPER_VOICE  --alien
"""
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import wave

from .protocol import MOTION_IP

# For play_url's yt-dlp: an HTTP proxy if the robot has no internet of its own. Empty = direct.
PROXY = os.environ.get('TALK_PROXY', '')


class TalkError(RuntimeError):
    pass


# --- speaker ------------------------------------------------------------------
HOST = 'ysc@' + MOTION_IP          # motion computer, where the speaker is
CLIP_DIR = '/home/ysc/lite3_voice'  # DEEP's own voice clips live here
RATE, CHANNELS = 48000, 2           # the codec's known-good playback format
# The amp swallows the start of a stream while it unmutes (a 1.1 s clip was
# inaudible). Raise it if a first syllable is ever clipped.
LEAD_IN_MS = 800
# Straight to the ES8388. ysc's PulseAudio default is a null sink, so a
# plain `aplay` "succeeds" silently. Needs ysc in the audio group on .120.
DEVICE = 'plughw:0,0'

# play_url(): standalone aarch64 binaries copied in from the laptop (no pip
# or github from here). yt-dlp goes out via TALK_PROXY when one is set.
YTDLP = os.path.expanduser('~/.local/bin/yt-dlp')
DENO = os.path.expanduser('~/.local/bin/deno')    # yt-dlp's JS runtime for YouTube

# say(): Piper, the standalone aarch64 release copied in from the laptop
# (2026-09-18). ~/piper is not on PATH for non-interactive ssh, so it is found
# here by absolute path. Voices are ~/piper/<name>.onnx (+ .onnx.json).
PIPER = os.path.expanduser('~/piper/piper')
PIPER_DIR = os.path.expanduser('~/piper')
PIPER_VOICE = 'en_US-ryan-medium'    # his voice
# Low noise = flat intonation and even timing: the "translation computer" sound.
PIPER_ARGS = ['--noise_scale', '0.1', '--noise_w', '0.1', '--length_scale', '1.1']
# Loudness: the codec PCM volume goes to max (192 = 0 dB) and speech gets a
# software gain with a limiter. SOFT_GAIN = 1.0 undoes the software part.
HW_PCM = 192
SOFT_GAIN = 1.6
# Master volume, 0.0 - 1.0, on top of the above (the HMI slider sets this).
# Read when a sound starts, so a change applies from the next sound on.
VOLUME = 1.0
# alien=True: thin, slightly raised, faintly metallic small-speaker voice.
ALIEN_FILTER = ('rubberband=pitch=1.12,highpass=f=300,lowpass=f=5000,'
                'aecho=0.8:0.6:12:0.2')


def _ssh(host):
    # No interactive prompt: rely on the installed key, and fail fast if it
    # ever stops working rather than hang on a password ask.
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host]


def _aplay():
    """aplay on the robot, playing the raw samples _to_speaker() makes."""
    return ['aplay', '-q', '-D', DEVICE, '-f', 'S16_LE', '-r', str(RATE),
            '-c', str(CHANNELS), '-']


def _to_speaker(*filters):
    """ffmpeg output arguments: these filters (None is skipped), then the amp
    lead-in, as raw samples on stdout."""
    lead = 'adelay=%d|%d' % (LEAD_IN_MS, LEAD_IN_MS)
    return ['-af', ','.join(f for f in filters + (lead,) if f),
            '-f', 's16le', '-ar', str(RATE), '-ac', str(CHANNELS), '-']


class Voice:
    """Sound out of the robot. One SSH hop per sound; nothing stays running."""

    def __init__(self, host=HOST, clip_dir=CLIP_DIR):
        self.host = host
        self.clip_dir = clip_dir
        self._ssh = _ssh(host)

    # --- the built-in clips ------------------------------------------------
    def clips(self):
        """Names (without .wav) of the clips already on the robot."""
        out = subprocess.run(self._ssh + ['ls', '-1', self.clip_dir],
                             capture_output=True, text=True)
        if out.returncode != 0:
            raise TalkError('could not list %s on %s: %s'
                             % (self.clip_dir, self.host, out.stderr.strip()))
        return sorted(f[:-4] for f in out.stdout.split() if f.endswith('.wav'))

    def play(self, name, wait=True):
        """Play one of the robot's own clips by name, e.g. play('okstop').
        Case-insensitive, .wav optional. Plays entirely on the robot.
        """
        clips = self.clips()
        key = name[:-4] if name.lower().endswith('.wav') else name
        match = next((c for c in clips if c.lower() == key.lower()), None)
        if match is None:
            raise TalkError('no clip %r. Available: %s'
                             % (name, ', '.join(clips)))
        path = '%s/%s.wav' % (self.clip_dir, match)
        # -q so aplay does not print the format banner on every call.
        cmd = self._ssh + ['aplay', '-q', '-D', DEVICE, path]
        if not wait:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            return None
        out = subprocess.run(cmd, capture_output=True, text=True)
        if out.returncode != 0:
            raise TalkError('play %s failed: %s' % (match, out.stderr.strip()))
        return True

    # --- arbitrary local audio ---------------------------------------------
    def play_file(self, path, wait=True):
        """Play any local audio file out of the robot: ffmpeg here converts it and
        streams the raw samples over ssh into aplay.
        """
        return self._stream(path, wait, 'play_file %s' % path)

    # --- speech ------------------------------------------------------------
    def say(self, text, wait=True, voice=None, alien=False):
        """Speak exactly `text`, through Speaker (streaming Piper).
        `voice` picks a Piper model (see voices()); alien=True is Rocky's sound.
        """
        if not wait:
            threading.Thread(target=self.say, args=(text,),
                             kwargs={'voice': voice, 'alien': alien}).start()
            return None
        with Speaker(voice=voice, alien=alien, flat=alien, host=self.host) as sp:
            for s in sentences([text + ' ']):
                sp.say(s)
            sp.finish()
        return True

    # --- internet audio ----------------------------------------------------
    def play_url(self, url, wait=True):
        """Play the audio of a YouTube (or any yt-dlp-supported) URL."""
        if not os.path.exists(YTDLP):
            raise TalkError('yt-dlp not found at %s' % YTDLP)
        ytdlp = [YTDLP] + (['--proxy', PROXY] if PROXY else []) + [
                 '--js-runtimes', 'deno:' + DENO,
                 '--no-playlist', '-f', 'bestaudio', '-q', '--no-warnings',
                 '-o', '-', url]
        return self._stream('-', wait, 'play_url %s' % url, pre=ytdlp)

    # --- plumbing ----------------------------------------------------------
    def voices(self):
        """Names of the installed Piper voices."""
        return sorted(os.path.basename(f)[:-5]
                      for f in glob.glob(os.path.join(PIPER_DIR, '*.onnx')))

    def _stream(self, src, wait, label, pre=None):
        """Run (pre | ffmpeg | ssh aplay -), ffmpeg converting to the codec's
        format. src is a file path, or '-' for what the command `pre` writes."""
        if not shutil.which('ffmpeg'):
            raise TalkError('ffmpeg not found on this machine - needed to '
                             'convert audio to the robot codec format')
        ff = (['ffmpeg', '-loglevel', 'error', '-i', src]
              + _to_speaker('volume=%.2f' % VOLUME))
        sink = self._ssh + _aplay()
        p_pre = subprocess.Popen(pre, stdout=subprocess.PIPE) if pre else None
        p_ff = subprocess.Popen(
            ff, stdin=(p_pre.stdout if p_pre else None),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if p_pre:
            p_pre.stdout.close()      # let ffmpeg own the read end
        p_ap = subprocess.Popen(sink, stdin=p_ff.stdout,
                               stderr=subprocess.PIPE)
        p_ff.stdout.close()
        if not wait:
            return None
        ap_err = p_ap.communicate()[1]
        ff_err = p_ff.communicate()[1]
        # Check the producer first: when yt-dlp dies, ffmpeg
        # only sees empty input and its error would hide the real cause.
        if p_pre and p_pre.wait() != 0:
            raise TalkError('%s: %s failed (exit %d), see its message above'
                             % (label, os.path.basename(pre[0]), p_pre.returncode))
        if p_ap.returncode != 0:
            raise TalkError('%s: aplay on robot failed: %s'
                             % (label, ap_err.decode(errors='replace').strip()))
        if p_ff.returncode not in (0, None):
            raise TalkError('%s: ffmpeg failed: %s'
                             % (label, ff_err.decode(errors='replace').strip()))
        return True


class Speaker:
    """Streaming Piper: say() queues ONE sentence and returns at once, finish()
    blocks until all of it has played. Piper stays running between calls; the
    speaker is opened at the first say() and released in finish().

        with Speaker(alien=True) as sp:
            sp.say('Hello friend.'); sp.say('You are well, question?')
            sp.finish()
    """

    def __init__(self, voice=None, alien=False, flat=True, host=HOST):
        """flat=True uses PIPER_ARGS (monotone); False is Piper's own
        expressive delivery."""
        model = os.path.join(PIPER_DIR, (voice or PIPER_VOICE) + '.onnx')
        if not os.access(PIPER, os.X_OK) or not os.path.exists(model):
            raise TalkError('Speaker needs piper at %s and voice %s' % (PIPER, model))
        self.rate = json.load(open(model + '.json'))['audio']['sample_rate']
        self.af = ALIEN_FILTER if alien else None
        self._ssh = _ssh(host)
        self._dir = tempfile.mkdtemp(prefix='speaker-')
        self._piper = subprocess.Popen(
            [PIPER, '-q', '--model', model] + (PIPER_ARGS if flat else [])
            + ['--output_dir', self._dir],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, universal_newlines=True, bufsize=1)
        self._cv = threading.Condition()
        self._pending = 0
        self._player = None
        self._error = None
        threading.Thread(target=self._pump, daemon=True).start()

    def say(self, sentence):
        text = ' '.join(sentence.split())      # piper reads one line per utterance
        if not text:
            return
        with self._cv:
            if self._error:
                raise TalkError(self._error)
            if self._player is None:
                self._player = self._open_player()
            self._pending += 1
        self._piper.stdin.write(text + '\n')
        self._piper.stdin.flush()

    def finish(self, timeout=120):
        """Wait until everything said so far has played, then free the speaker."""
        with self._cv:
            if not self._cv.wait_for(lambda: self._pending == 0 or self._error, timeout):
                self._error = 'speaker timed out'
            player, self._player = self._player, None
            err = self._error
        if player:
            ff, ap = player
            ff.stdin.close()
            ff.wait()
            if ap.wait() != 0 and not err:
                err = 'aplay on the robot failed (exit %d)' % ap.returncode
        if err:
            raise TalkError(err)

    def close(self):
        try:
            self._piper.stdin.close()
        except OSError:
            pass
        self._piper.wait()
        shutil.rmtree(self._dir, ignore_errors=True)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _open_player(self):
        gain = 'volume=%.2f,alimiter=limit=0.95' % (SOFT_GAIN * VOLUME)
        # One stream serves a whole reply, so the amp lead-in costs LEAD_IN_MS
        # once per reply, not per line.
        ff = (['ffmpeg', '-loglevel', 'error', '-f', 's16le', '-ar', str(self.rate),
               '-ac', '1', '-i', '-'] + _to_speaker(self.af, gain))
        p_ff = subprocess.Popen(ff, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        p_ap = subprocess.Popen(
            self._ssh + ['amixer', '-q', '-c', '0', 'sset', 'PCM', str(HW_PCM), ';']
            + _aplay(), stdin=p_ff.stdout,
            # the gap between sentences makes aplay print "underrun!!!", which
            # is expected here; real failures show up as its exit code.
            stderr=subprocess.DEVNULL)
        p_ff.stdout.close()
        return p_ff, p_ap

    def _pump(self):
        """Feed each finished wav into the open player, in order."""
        for line in self._piper.stdout:
            path = line.strip()
            try:
                with wave.open(path) as w:
                    frames = w.readframes(w.getnframes())
                os.remove(path)
                with self._cv:
                    player = self._player
                if player:
                    player[0].stdin.write(frames)
                    player[0].stdin.flush()
            except (OSError, EOFError, wave.Error) as e:
                with self._cv:
                    self._error = 'speaker: %s' % e
            with self._cv:
                self._pending -= 1
                self._cv.notify_all()
        with self._cv:
            if self._pending:
                self._error = 'piper exited unexpectedly'
            self._cv.notify_all()


_END = re.compile(r'[.!?]["\')]*\s')


def sentences(chunks):
    """Regroup a stream of text chunks into whole sentences."""
    buf = ''
    for c in chunks:
        buf += c
        while True:
            m = _END.search(buf)
            if not m:
                break
            yield buf[:m.end()].strip()
            buf = buf[m.end():]
    if buf.strip():
        yield buf.strip()


def _cli(argv):
    voice, alien, args, it = None, False, [], iter(argv)
    for a in it:
        if a in ('-h', '--help', 'help'):
            print(__doc__); return 0
        elif a == '--alien':
            alien = True
        elif a == '--voice':
            voice = next(it)
        else:
            args.append(a)
    if not args:
        print(__doc__); return 2
    cmd, rest = args[0], ' '.join(args[1:])
    try:
        v = Voice()
        if cmd == 'say':
            v.say(rest, voice=voice, alien=alien)
        elif cmd == 'play':
            v.play(rest)
        elif cmd == 'file':
            v.play_file(rest)
        elif cmd == 'url':
            v.play_url(rest)
        elif cmd == 'clips':
            print('\n'.join(v.clips()))
        elif cmd == 'voices':
            print('\n'.join(v.voices()))
        else:
            print('unknown command %r - see --help' % cmd, file=sys.stderr)
            return 2
    except TalkError as e:
        print('error: %s' % e, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(_cli(sys.argv[1:]))
