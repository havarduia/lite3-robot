#!/usr/bin/env python3
"""frontcam - the front camera's frames during a run, each with its arrival time.

    python3 frontcam.py <out_dir> <seconds> [per_second]     2 a second unless told
    python3 frontcam.py check                                 self-check, no camera

bin/run.py copies this one file to the Orin (it decodes the stream there:
60% of a core on the Orin, which has room; the Jetson has not). Every frame
is read, so the stream's buffer never goes stale and an arrival time is
when the frame really came; only `per_second` of them are saved, as
out_dir/NNNNN.jpg, with "arrival_time NNNNN.jpg" lines in out_dir/frames.txt.
The camera's own delay (env/front_camera.yaml delay_s, 0.29 s) is not taken
off here: bin/report.py does that. OpenCV stamps the frames: ffmpeg's own
timestamps on this stream are unusable (LIDAR_ORIN_LOG.md section 29).
"""
import os
import signal
import sys
import time

STREAM = 'rtsp://192.168.1.120:8554/test'
JPEG_QUALITY = 85


def record(read, out_dir, seconds, per_second, clock=time.time, write=None):
    """Save every frame that falls due, at most per_second a second, until `seconds` pass
    or read() gives nothing. read() -> image or None; write(path, image). Returns how many."""
    os.makedirs(out_dir, exist_ok=True)
    index = open(os.path.join(out_dir, 'frames.txt'), 'a')
    end, due, n = clock() + seconds, 0.0, 0
    while clock() < end:
        frame = read()
        if frame is None:
            break
        t = clock()
        if t >= due:
            name = '%05d.jpg' % n
            write(os.path.join(out_dir, name), frame)
            index.write('%.4f %s\n' % (t, name))
            index.flush()
            n += 1
            due = t + 1.0 / per_second      # from this frame: after a stall, the burst that follows is not all saved
    index.close()
    return n


def main(out_dir, seconds, per_second=2.0):
    import cv2
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    cap = cv2.VideoCapture(STREAM, cv2.CAP_FFMPEG)

    def read():
        ok, frame = cap.read()
        return frame if ok else None

    n = record(read, out_dir, float(seconds), float(per_second),
               write=lambda p, f: cv2.imwrite(p, f, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]))
    print('frontcam: %d frames saved' % n)


def check():
    import tempfile
    now = [0.0]

    def clock():
        return now[0]

    def read():                     # a camera at 30 frames a second
        now[0] += 1.0 / 30
        return 'frame'

    written = []
    with tempfile.TemporaryDirectory() as d:
        n = record(read, d, 10.0, 2.0, clock=clock, write=lambda p, f: written.append(p))
        lines = open(os.path.join(d, 'frames.txt')).read().split('\n')
    assert 19 <= n <= 21 and len(written) == n, (n, len(written))      # about 2 a second for 10 s
    times = [float(l.split()[0]) for l in lines if l]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert all(0.5 - 1e-3 <= g <= 0.5 + 1.0 / 30 + 1e-3 for g in gaps), gaps     # half a second, plus up to a frame (times kept to 0.1 ms)
    assert lines[0].split()[1] == '00000.jpg' and written[0].endswith('00000.jpg')
    frames = iter(['a', 'b', None])                                     # the stream ends: it stops
    with tempfile.TemporaryDirectory() as d:
        assert record(lambda: next(frames), d, 10.0, 2.0, clock=clock, write=lambda p, f: None) == 1
    print('frontcam: check passed')


if __name__ == '__main__':
    if sys.argv[1:] == ['check']:
        check()
    elif len(sys.argv) in (3, 4):
        main(*sys.argv[1:])
    else:
        sys.exit(__doc__)
