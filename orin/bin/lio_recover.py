#!/usr/bin/env python3
"""Rebuild a saved map from a Live session that never got to Stop & save.

A battery swap or power cut kills lio_live.py before it can save, but its
per-scan log (scans.bin: sensor position + the 10 cm cells each scan hit)
is on disk as it goes. That is enough to rebuild the map and the path:

    lio-recover ~/maps/live --name long-walk         -> ~/maps/long-walk_<stamp>/
    lio-recover <dir with scans.bin> --name x --no-post

Then, unless --no-post: ghost removal (lio_clean.py), floor maps
(map_walls.py) and the 3D viewer (map_web.py), as Stop & save would.

What a recovered map lacks compared with a normal save:
  - FAST-LIO's final gravity estimate (/lio/gravity): the map stays levelled
    by the first second of the run, which is right if the robot started
    standing still
  - orientations: poses get identity rotation; only positions are logged
  - the last few seconds before the cut, which were still in a write buffer
Copy scans.bin somewhere safe BEFORE the next Live start: it starts a new one.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

import numpy as np

BIN = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, BIN)
from maplib import centres, read_scans, write_ply  # noqa: E402


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                 description=__doc__.split("\n\n")[0])
    ap.add_argument("src", help="directory holding scans.bin (e.g. ~/maps/live)")
    ap.add_argument("--name", default="recovered")
    ap.add_argument("--stamp", default=None, help="default: scans.bin's start time")
    ap.add_argument("--no-post", action="store_true")
    a = ap.parse_args()
    src = os.path.expanduser(a.src)
    sf = os.path.join(src, "scans.bin")
    if not os.path.exists(sf):
        sys.exit(f"error: no scans.bin in {src}")
    scans = read_scans(sf)
    if len(scans) < 20:
        sys.exit("error: fewer than 20 scans - nothing worth saving")
    O = np.array([o for o, _ in scans])
    keys = np.unique(np.concatenate([k for _, k in scans]))
    stamp = a.stamp or time.strftime(
        "%Y%m%d_%H%M%S", time.localtime(os.path.getmtime(sf) - len(scans) / 10.0))
    name = f"{a.name}_{stamp}"
    top = os.path.join(os.path.expanduser("~/maps"), name)
    out = os.path.join(top, "slam_output", "latest")
    if os.path.exists(top):
        sys.exit(f"error: {top} already exists")
    os.makedirs(out)
    pts = centres(keys)
    write_ply(os.path.join(out, "gravity_aligned.ply"), np.column_stack([pts, np.zeros(len(pts))]))
    P = np.tile(np.eye(4), (len(O), 1, 1))
    P[:, :3, 3] = O
    np.save(os.path.join(out, f"{name}_poses.npy"), P)
    np.save(os.path.join(out, "gravity_aligned_path.npy"), O.astype(np.float32))
    np.save(os.path.join(out, "relevel.npy"), np.eye(3))
    shutil.copy(sf, os.path.join(out, "scans.bin"))
    for f in ("fastlio.log",):
        if os.path.exists(os.path.join(src, f)):
            shutil.copy(os.path.join(src, f), os.path.join(out, f))
    d = np.linalg.norm(np.diff(O[:, :2], axis=0), axis=1).sum()
    print(f"recovered {len(scans)} scans (~{len(scans) / 600:.1f} min, {d:.0f} m), "
          f"{len(pts):,} points -> {top}", flush=True)
    if a.no_post:
        return
    py = "/usr/bin/python3"
    subprocess.run([py, os.path.join(BIN, "lio_clean.py"), top, "--no-rebuild"])
    subprocess.run([py, os.path.join(BIN, "map_walls.py"), top])
    subprocess.run([py, os.path.join(BIN, "map_web.py"), top, "-o",
                    os.path.join(os.path.dirname(top), name + ".html"), "--title", name])


if __name__ == "__main__":
    main()
