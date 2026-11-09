#!/usr/bin/env python3
"""tts_server - his voice: Kokoro text-to-speech on the Orin's GPU (docs/voice.md).

robot/talk.py (RemoteSpeaker) on the perception computer sends one sentence per line
over TCP. Each answer is a little-endian uint32 byte count, then that many bytes of
16-bit mono PCM at RATE Hz. The model stays loaded, so a sentence costs ~0.4 s
(Piper on the Xavier: 0.3-1.7 s, and the user disliked its voice, 2026-11-09).

Listens on the robot's cable network only, so nobody on campus Wi-Fi can make him talk.
Runs in ~/tts/venv (kokoro-onnx, onnxruntime-gpu from the Jetson AI Lab index) with the
model in ~/tts/models; env/kokoro-tts.service starts it.

    ~/tts/venv/bin/python ~/bin/tts_server.py           # serve
    ~/tts/venv/bin/python ~/bin/tts_server.py --check   # say "Hello." once, timed, no server
"""
import os
import socket
import struct
import sys
import threading
import time

import numpy as np
import onnxruntime as ort
from kokoro_onnx import Kokoro

BIND = (os.environ.get('TTS_BIND', '192.168.1.5'), 8047)
RATE = 24000                                        # Kokoro's own; talk.py's TTS_RATE must match
VOICE = os.environ.get('TTS_VOICE', 'bm_george')    # the user's pick, 2026-11-09
SPEED = float(os.environ.get('TTS_SPEED', '1.0'))
LANG = 'en-gb' if VOICE.startswith('b') else 'en-us'    # b* voices are British
MODELS = os.path.expanduser('~/tts/models/')


def load():
    sess = ort.InferenceSession(MODELS + 'kokoro-v1.0.onnx',
                                providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
    k = Kokoro.from_session(sess, MODELS + 'voices-v1.0.bin')
    k.create('Warm up.', voice=VOICE, speed=SPEED, lang=LANG)    # the first call builds the GPU kernels
    print('tts_server: %s on %s' % (VOICE, sess.get_providers()[0]), flush=True)
    return k


def pcm(k, text):
    wav, sr = k.create(text, voice=VOICE, speed=SPEED, lang=LANG)
    assert sr == RATE, sr
    return (np.clip(wav, -1.0, 1.0) * 32767).astype('<i2').tobytes()


def serve(k):
    gpu = threading.Lock()                          # one sentence at a time on the GPU

    def client(c, who):
        with c, c.makefile('rb') as f:
            for line in f:
                text = line.decode('utf-8', 'replace').strip()
                try:
                    with gpu:
                        data = pcm(k, text) if text else b''
                except Exception as e:              # a bad sentence costs that client, not the server
                    print('tts_server: %r failed: %s' % (text[:60], e), flush=True)
                    return
                c.sendall(struct.pack('<I', len(data)) + data)
        print('tts_server: %s left' % who, flush=True)

    srv = socket.create_server(BIND)
    print('tts_server: listening on %s:%d' % BIND, flush=True)
    while True:
        c, who = srv.accept()
        c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=client, args=(c, who[0]), daemon=True).start()


if __name__ == '__main__':
    k = load()
    if '--check' in sys.argv:
        t = time.time()
        n = len(pcm(k, 'Hello.')) // 2
        print('tts_server check: "Hello." %.2f s for %.2f s of audio' % (time.time() - t, n / RATE))
    else:
        serve(k)
