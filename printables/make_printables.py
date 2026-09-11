# /// script
# requires-python = ">=3.10"
# dependencies = ["opencv-contrib-python==4.10.0.84", "numpy<2", "reportlab"]
# ///
"""Generate the print-ready PDFs for camera calibration and marker tracking.

Run from the corvid/ folder on the HOST (not in the container):

    uv run printables/make_printables.py

Two sheets come out, both A3, both at true metric scale:

    aruco_4x4_id0_200mm.pdf     the tag the drone tracks
    charuco_7x10_35mm.pdf       the target the camera is calibrated against

WHY THE PATTERNS ARE RENDERED BY OPENCV AND NOT DRAWN HERE
----------------------------------------------------------
OpenCV ships its own pattern generator (apps/pattern-tools/generate_pattern.py)
which re-implements the ChArUco layout in SVG. That re-implementation is exactly
where opencv#23152 came from: OpenCV 4.6 changed the board layout, boards printed
from one version stopped being detectable by another, and nothing warned anybody.

So both patterns here are rendered by the OpenCV library itself -- the same code
that will later detect them -- and then placed on the page at an exact physical
size. The layout cannot disagree with the detector because it IS the detector's
own layout. Every run then re-detects its own render before writing the PDF, so
a mismatch is caught on this machine rather than on a printed A3 sheet in a lab.

WHY THE BOARD IS CHARUCO AND NOT A PLAIN CHECKERBOARD
-----------------------------------------------------
A plain checkerboard has to be fully visible in every calibration image, so the
target can never be pushed into the corners of the frame -- which is where lens
distortion is largest and where the distortion coefficients are actually
determined. Every white square of a ChArUco board carries a uniquely coded
marker, so partial views still contribute. calib.io recommend it for exactly
this reason, along with the rule followed below: one board dimension odd, the
other even, so the board cannot be mistaken for its own 180-degree rotation.

WHY THE BOARD USES A DIFFERENT DICTIONARY FROM THE TAG
-------------------------------------------------------
The tracking tag is DICT_4X4_50 id 0. The board uses DICT_5X5_100. If the board
drew from the same dictionary its 35 coded squares would include id 0, and a
calibration board lying in shot during a flight test would publish detections
the station keeper would happily fly towards. Different dictionaries make that
impossible rather than unlikely.

SCALE
-----
Nothing here can stop a printer scaling the page, so nothing here tries. Both
sheets carry a printed 100 mm ruler. Measure it after printing:

  - The BOARD's scale barely matters. Camera intrinsics come out of the ratios
    between corners, so a uniformly scaled board yields identical fx, fy, cx, cy
    and distortion coefficients -- only the board's estimated distance changes,
    and we throw that away. Measure the square anyway and feed the measured
    number in; it costs nothing.
  - The TAG's scale matters directly and linearly. solvePnP takes the marker
    side length in metres, so a tag printed 1% small reports every range 1%
    short -- 2.3 cm at our hold height, which is the whole error budget. Measure
    the tag and put the MEASURED side length into the marker_size_m parameter
    rather than assuming 0.200.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import cv2
import numpy as np
from reportlab.lib.pagesizes import A3
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas as pdfcanvas

# --- the tracking tag ------------------------------------------------------
# Matches models/aruco_marker/ exactly: DICT_4X4_50, id 0, and a quiet zone one
# cell wide. The marker grid is 6 cells (4 data + 1 black border each side), so
# the quiet zone is 200/6 mm and the printed square is 200 * 8/6 = 266.67 mm.
TAG_DICT = cv2.aruco.DICT_4X4_50
TAG_ID = 0
TAG_SIDE_MM = 200.0
TAG_GRID_CELLS = 6
TAG_QUIET_CELLS = 1
TAG_PX_PER_CELL = 525            # 525 * 8 = 4200 px, ~400 dpi, integer cells

# --- the calibration board -------------------------------------------------
# 7 x 10: one dimension odd, one even, per calib.io, so the board has no
# 180-degree ambiguity. 35 mm squares put a 245 x 350 mm board on A3 with a
# comfortable margin. The marker is 0.75 of the square, which leaves a white
# border of 4.375 mm around each marker -- 117% of one 5x5 marker bit, above
# OpenCV's own 70% stability warning threshold.
BOARD_DICT = cv2.aruco.DICT_5X5_100
BOARD_COLS = 7
BOARD_ROWS = 10
BOARD_SQUARE_MM = 35.0
BOARD_MARKER_MM = 26.25
BOARD_PX_PER_SQUARE = 560        # 560/35*26.25 = 420 px marker = 60 px per bit

OUT_DIR = Path(__file__).resolve().parent


def _draw_ruler(c: pdfcanvas.Canvas, x_mm: float, y_mm: float) -> None:
    """A printed 100 mm ruler, ticked every 10 mm, tall tick at 50 and 100.

    This is the only defence against a scaled print, because it is the only
    thing on the page whose true length you already know.
    """
    c.setLineWidth(0.6)
    c.line(x_mm * mm, y_mm * mm, (x_mm + 100.0) * mm, y_mm * mm)
    for i in range(11):
        tick = 5.0 if i % 5 else 8.0
        tx = (x_mm + i * 10.0) * mm
        c.line(tx, y_mm * mm, tx, (y_mm * mm) + tick * mm / 3.0)
    c.setFont("Helvetica", 8)
    c.drawString(x_mm * mm, (y_mm - 4.0) * mm, "0")
    c.drawRightString((x_mm + 100.0) * mm, (y_mm - 4.0) * mm, "100 mm")


def _place(c: pdfcanvas.Canvas, img: np.ndarray, w_mm: float, h_mm: float,
           y_mm: float, tmp: Path) -> None:
    """Centre a rendered pattern on the page at an exact physical size."""
    page_w_mm = A3[0] / mm
    cv2.imwrite(str(tmp), img)
    c.drawImage(str(tmp), ((page_w_mm - w_mm) / 2.0) * mm, y_mm * mm,
                width=w_mm * mm, height=h_mm * mm)


def make_tag(tmp_dir: Path) -> Path:
    d = cv2.aruco.getPredefinedDictionary(TAG_DICT)

    # Rendered by OpenCV at an exact whole number of pixels per marker cell, so
    # no cell boundary lands mid-pixel and nothing is interpolated.
    marker_px = TAG_PX_PER_CELL * TAG_GRID_CELLS
    pad_px = TAG_PX_PER_CELL * TAG_QUIET_CELLS
    marker = cv2.aruco.generateImageMarker(d, TAG_ID, marker_px)

    total_px = marker_px + 2 * pad_px
    img = np.full((total_px, total_px), 255, dtype=np.uint8)
    img[pad_px:pad_px + marker_px, pad_px:pad_px + marker_px] = marker

    # Verify before printing: detect the render with the SAME refinement method
    # the flight node uses, and confirm it is the id we asked for.
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    corners, ids, _ = cv2.aruco.ArucoDetector(d, params).detectMarkers(img)
    if ids is None or TAG_ID not in ids.flatten():
        raise SystemExit("FAIL: rendered tag does not detect as id %d" % TAG_ID)

    quiet_mm = TAG_SIDE_MM / TAG_GRID_CELLS
    total_mm = TAG_SIDE_MM + 2 * quiet_mm

    out = OUT_DIR / "aruco_4x4_id0_200mm.pdf"
    c = pdfcanvas.Canvas(str(out), pagesize=A3)
    page_h_mm = A3[1] / mm
    y = (page_h_mm - total_mm) / 2.0 + 12.0
    _place(c, img, total_mm, total_mm, y, tmp_dir / "tag.png")

    c.setFont("Helvetica-Bold", 10)
    c.drawCentredString(A3[0] / 2.0, (y + total_mm + 10.0) * mm,
                        "corvid tracking tag - DICT_4X4_50 id 0")
    c.setFont("Helvetica", 8.5)
    c.drawCentredString(A3[0] / 2.0, (y + total_mm + 4.0) * mm,
                        "PRINT AT 100%% - NOT 'fit to page'.  Black marker %.0f mm square, "
                        "%.1f mm including the white quiet zone." % (TAG_SIDE_MM, total_mm))
    c.setFont("Helvetica", 8)
    c.drawCentredString(A3[0] / 2.0, (y - 8.0) * mm,
                        "Measure the BLACK SQUARE, edge to edge, and put that number in "
                        "marker_size_m. A 1% error here is a 1% range error.")
    _draw_ruler(c, (A3[0] / mm - 100.0) / 2.0, y - 20.0)
    c.showPage()
    c.save()
    return out


def make_board(tmp_dir: Path) -> Path:
    d = cv2.aruco.getPredefinedDictionary(BOARD_DICT)
    board = cv2.aruco.CharucoBoard(
        (BOARD_COLS, BOARD_ROWS), BOARD_SQUARE_MM / 1000.0,
        BOARD_MARKER_MM / 1000.0, d)

    w_px = BOARD_PX_PER_SQUARE * BOARD_COLS
    h_px = BOARD_PX_PER_SQUARE * BOARD_ROWS
    img = board.generateImage((w_px, h_px))

    # Verify before printing: every interior chessboard corner must come back.
    expected = (BOARD_COLS - 1) * (BOARD_ROWS - 1)
    ch_corners, ch_ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(img)
    found = 0 if ch_ids is None else len(ch_ids)
    if found != expected:
        raise SystemExit(
            "FAIL: rendered board detects %d of %d corners under OpenCV %s"
            % (found, expected, cv2.__version__))

    w_mm = BOARD_SQUARE_MM * BOARD_COLS
    h_mm = BOARD_SQUARE_MM * BOARD_ROWS

    out = OUT_DIR / "charuco_7x10_35mm.pdf"
    c = pdfcanvas.Canvas(str(out), pagesize=A3)
    page_h_mm = A3[1] / mm
    y = (page_h_mm - h_mm) / 2.0 + 8.0
    _place(c, img, w_mm, h_mm, y, tmp_dir / "board.png")

    c.setFont("Helvetica-Bold", 10)
    c.drawCentredString(A3[0] / 2.0, (y + h_mm + 9.0) * mm,
                        "corvid calibration board - ChArUco %dx%d, DICT_5X5_100"
                        % (BOARD_COLS, BOARD_ROWS))
    c.setFont("Helvetica", 8.5)
    c.drawCentredString(A3[0] / 2.0, (y + h_mm + 3.5) * mm,
                        "PRINT AT 100%%.  Square %.1f mm, marker %.2f mm, board %.0f x %.0f mm."
                        % (BOARD_SQUARE_MM, BOARD_MARKER_MM, w_mm, h_mm))
    c.setFont("Helvetica", 8)
    c.drawCentredString(A3[0] / 2.0, (y - 7.0) * mm,
                        "Mount FLAT - glass or foam board. Flatness matters far more than "
                        "scale here; intrinsics are scale-invariant.")
    _draw_ruler(c, (A3[0] / mm - 100.0) / 2.0, y - 19.0)
    c.showPage()
    c.save()
    return out


def main() -> None:
    print("OpenCV %s" % cv2.__version__)
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        tag = make_tag(tmp)
        print("  OK  tag detects as DICT_4X4_50 id %d" % TAG_ID)
        print("      wrote %s" % tag.name)
        board = make_board(tmp)
        print("  OK  board detects all %d interior corners"
              % ((BOARD_COLS - 1) * (BOARD_ROWS - 1)))
        print("      wrote %s" % board.name)


if __name__ == "__main__":
    main()
