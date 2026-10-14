#!/usr/bin/env python3
"""AprilTags for measuring runs: the sheets to print, and finding them in a picture. Laptop only.

    bin/tags.py make [out_dir]       tags 0-9 (AprilTag 36h11) as an A4 PDF, 14 cm, with a 10 cm scale bar
    bin/tags.py board [out_dir]      a ChArUco board (A4) for calibrating the front camera
    bin/tags.py check                self-check: a tag drawn at a known pose is found at that pose

Print at 100% ("actual size"), then measure the scale bar: it must be 10.0 cm,
and the black square of each tag TAG_M. Glue them flat on stiff board.

Written for the laptop's OpenCV 4.6 (the cv2.aruco calls before 4.7 renamed
them). The camera's numbers come from env/front_camera.yaml once task 8d of
.planning/plans/compute-distribution.md has filled it in. README "Measuring a run".
"""
import math
import os
import sys

import cv2
import numpy as np

ARUCO = cv2.aruco
FAMILY = ARUCO.getPredefinedDictionary(ARUCO.DICT_APRILTAG_36h11)
TAG_M = 0.14            # black square, edge to edge
# White all round, at least this many cells: with one cell the detector misses
# the tag on any background darker than paper (found in check()). On A4, 14 cm
# leaves exactly 2 cells of paper either side: do not trim the sheet.
MARGIN_CELLS = 2
IDS = range(10)
A4_MM = (210, 297)
DPI = 300
BOARD = (5, 7, 0.035, 0.026)    # squares across, down, square and marker size in m: 175 x 245 mm on A4


def tag_image(tag_id, px, margin_cells=MARGIN_CELLS):
    """The tag's black square, px wide, with margin_cells of white all round."""
    cell = round(px / (FAMILY.markerSize + 2))
    img = ARUCO.drawMarker(FAMILY, tag_id, cell * (FAMILY.markerSize + 2), borderBits=1)
    m = margin_cells * cell
    return cv2.copyMakeBorder(img, m, m, m, m, cv2.BORDER_CONSTANT, value=255)


def page(draw):
    """An A4 page at DPI as a white uint8 image, handed to draw(img, px_per_mm)."""
    ppmm = DPI / 25.4
    img = np.full((round(A4_MM[1] * ppmm), round(A4_MM[0] * ppmm)), 255, np.uint8)
    draw(img, ppmm)
    return img


def scale_bar(img, ppmm, x_mm, y_mm):
    x, y, n = round(x_mm * ppmm), round(y_mm * ppmm), round(100 * ppmm)
    cv2.rectangle(img, (x, y), (x + n, y + round(3 * ppmm)), 0, -1)
    cv2.putText(img, '10.0 cm', (x, y + round(10 * ppmm)), cv2.FONT_HERSHEY_SIMPLEX, 2.0, 0, 4)


def tag_page(tag_id):
    def draw(img, ppmm):
        # the paper round it is the white margin (2 cells either side on A4)
        t = tag_image(tag_id, round(TAG_M * 1000 * ppmm), margin_cells=0)
        x0, y0 = (img.shape[1] - t.shape[1]) // 2, round(20 * ppmm)
        img[y0:y0 + t.shape[0], x0:x0 + t.shape[1]] = t
        cv2.putText(img, 'AprilTag 36h11  id %d   black square %.0f cm' % (tag_id, TAG_M * 100),
                    (round(25 * ppmm), y0 + t.shape[0] + round(20 * ppmm)), cv2.FONT_HERSHEY_SIMPLEX, 2.2, 0, 5)
        scale_bar(img, ppmm, 25, A4_MM[1] - 45)
    return page(draw)


def board():
    across, down, square, marker = BOARD
    return ARUCO.CharucoBoard_create(across, down, square, marker, FAMILY)


def board_page():
    def draw(img, ppmm):
        across, down, square, _ = BOARD
        w, h = round(across * square * 1000 * ppmm), round(down * square * 1000 * ppmm)
        b = board().draw((w, h))
        x0, y0 = (img.shape[1] - w) // 2, round(8 * ppmm)
        img[y0:y0 + h, x0:x0 + w] = b
        cv2.putText(img, 'ChArUco %dx%d, square %.1f mm, marker %.1f mm, AprilTag 36h11' % (
            BOARD[0], BOARD[1], BOARD[2] * 1000, BOARD[3] * 1000),
            (round(25 * ppmm), y0 + h + round(9 * ppmm)), cv2.FONT_HERSHEY_SIMPLEX, 1.6, 0, 4)
        scale_bar(img, ppmm, 25, A4_MM[1] - 25)
    return page(draw)


def write_pdf(pages, path):
    """Pages as a PDF at true size: each page is A4 at DPI."""
    from PIL import Image
    first, *rest = [Image.fromarray(p) for p in pages]
    first.save(path, save_all=True, append_images=rest, resolution=DPI)


def detect(gray, K, dist, fisheye=False, size=TAG_M):
    """[(id, R, t, corners)] for every tag in an 8-bit picture: R, t place the tag in the camera's
    optical frame (x right, y down, z forward), t in m. Corners in pixels, as seen."""
    params = ARUCO.DetectorParameters_create()
    params.cornerRefinementMethod = ARUCO.CORNER_REFINE_SUBPIX
    corners, ids, _ = ARUCO.detectMarkers(gray, FAMILY, parameters=params)
    if ids is None:
        return []
    out = []
    for c, i in zip(corners, ids.ravel()):
        found = pose(c.reshape(4, 2), K, dist, fisheye, size)
        if found:
            out.append((int(i),) + found + (c.reshape(4, 2),))
    return out


def pose(corners, K, dist, fisheye=False, size=TAG_M):
    """(R, t) of a tag from its four corners in pixels, in detectMarkers' order; None if no fit."""
    h = size / 2    # the tag's corners in its own frame
    square = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    pts, d = corners.reshape(-1, 1, 2).astype(np.float64), dist
    if fisheye:     # straighten the corners first; solvePnP only knows the plain lens model
        pts, d = cv2.fisheye.undistortPoints(pts, K, dist, P=K), None
    ok, rvec, tvec = cv2.solvePnP(square, pts, K, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return (cv2.Rodrigues(rvec)[0], tvec.ravel()) if ok else None


def mount(x, y, z, roll, pitch, yaw):
    """base_link <- camera optical frame, 4x4, from where the camera sits on the body and how it
    is turned (rad, REP-103: pitch positive = nose down)."""
    def rot(axis, a):
        c, s = math.cos(a), math.sin(a)
        i, j = [(1, 2), (2, 0), (0, 1)][axis]
        r = np.eye(3)
        r[i, i], r[i, j], r[j, i], r[j, j] = c, -s, s, c
        return r
    optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)     # optical z forward -> body x forward
    T = np.eye(4)
    T[:3, :3] = rot(2, yaw) @ rot(1, pitch) @ rot(0, roll) @ optical
    T[:3, 3] = (x, y, z)
    return T


def in_base(T_base_cam, R, t):
    """A tag seen at R, t (optical frame) -> its pose in base_link, 4x4."""
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T_base_cam @ T


def render(tag_id, K, R, t, shape=(720, 1280)):
    """A synthetic camera picture of one tag at R, t: for the self-check."""
    px = 800
    img = tag_image(tag_id, px)
    cell = img.shape[0] // (FAMILY.markerSize + 2 + 2 * MARGIN_CELLS)
    m = MARGIN_CELLS * cell
    black = img.shape[0] - 2 * m            # the black square inside the white margin, in px
    src = np.array([[m, m], [m + black, m], [m + black, m + black], [m, m + black]], np.float32)
    h = TAG_M / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    dst = (K @ (R @ obj.T + t.reshape(3, 1))).T
    dst = (dst[:, :2] / dst[:, 2:]).astype(np.float32)
    H = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(img, H, (shape[1], shape[0]), borderValue=180)


def angle_between(Ra, Rb):
    return math.degrees(math.acos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


def check():
    K = np.array([[700.0, 0, 640], [0, 700, 360], [0, 0, 1]])
    dist = np.zeros(5)
    facing = np.diag([1.0, -1, -1])     # a tag's z comes out of its face: one facing the camera is turned 180 deg about x
    for distance, turn in [(0.6, 0), (1.5, 30), (3.0, 45)]:
        R = cv2.Rodrigues(np.array([0.1, math.radians(turn), 0.05]))[0] @ facing
        t = np.array([0.15, -0.05, distance])
        found = detect(render(7, K, R, t), K, dist)
        assert [f[0] for f in found] == [7], found
        _, Rf, tf, _ = found[0]
        err, deg = np.linalg.norm(tf - t), angle_between(R, Rf)
        # a 14 cm tag is ~33 px wide at 3 m with this lens: about 1% of the range is what it can give
        assert err < 0.005 + 0.01 * distance and deg < 1.0 + distance, (distance, turn, err, deg)
        print('  %.1f m, turned %2d deg: off by %.3f m, %.2f deg' % (distance, turn, err, deg))
    # a strongly bent fisheye lens: corners put through it come back at the pose they were made from
    R, t = cv2.Rodrigues(np.array([0.2, 0.4, 0]))[0] @ facing, np.array([0.6, 0.2, 1.2])
    h = TAG_M / 2
    obj = (R @ np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]]).T).T + t
    D = np.array([0.1, -0.05, 0.01, 0.0])
    bent = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)[0].reshape(4, 2)
    Rf, tf = pose(bent, K, D, fisheye=True)
    assert np.linalg.norm(tf - t) < 1e-4 and angle_between(R, Rf) < 0.05, (tf, t)
    assert np.linalg.norm(pose(bent, K, dist)[1] - t) > 0.01     # read as a plain lens, it is wrong
    # mounting: a camera level and facing forward sees a tag 2 m ahead as 2 m ahead of it on base_link
    T = mount(0.3, 0, 0.1, 0, 0, 0)
    ahead = in_base(T, np.eye(3), np.array([0, 0, 2.0]))[:3, 3]
    assert np.allclose(ahead, [2.3, 0, 0.1]), ahead
    # pitched 20 deg nose down: something straight along the lens axis is below the camera
    T = mount(0, 0, 0, 0, math.radians(20), 0)
    p = in_base(T, np.eye(3), np.array([0, 0, 1.0]))[:3, 3]
    assert np.allclose(p, [math.cos(math.radians(20)), 0, -math.sin(math.radians(20))]), p
    # the printed sheet: the black square of each tag is TAG_M wide at DPI
    pg = tag_page(0)
    rows = np.where((pg == 0).any(axis=1))[0]
    cols = np.where((pg[rows[0]] == 0))[0]
    mm = (cols[-1] - cols[0] + 1) / (DPI / 25.4)
    assert abs(mm - TAG_M * 1000) < 0.5, mm
    print('tags: check passed')


def main(argv):
    if argv == ['check']:
        return check()
    if argv and argv[0] in ('make', 'board'):
        out = argv[1] if len(argv) > 1 else '.'
        os.makedirs(out, exist_ok=True)
        if argv[0] == 'make':
            path = os.path.join(out, 'apriltags_36h11_0-9_%dcm.pdf' % round(TAG_M * 100))
            write_pdf([tag_page(i) for i in IDS], path)
        else:
            path = os.path.join(out, 'charuco_board_a4.pdf')
            write_pdf([board_page()], path)
        print('wrote %s; print at 100%% and check the scale bar is 10.0 cm' % path)
        return
    sys.exit(__doc__)


if __name__ == '__main__':
    main(sys.argv[1:])
