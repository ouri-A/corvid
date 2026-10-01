# /// script
# requires-python = ">=3.10"
# dependencies = ["opencv-contrib-python", "numpy"]
# ///
"""Generate the ArUco marker texture used by the Gazebo model.

Run from the corvid/ folder on the HOST (not in the container):

    uv run scripts/make_marker.py

The output PNG is the marker plus a white quiet zone one cell wide on every
side. The quiet zone is not decoration -- ArUco detection needs white space
around the black border or the tag will not be found at all.

Sizes are chosen so the marker occupies exactly 3/4 of the image width:

    marker      840 px  (6 cells of 140 px: 4 data cells + 1 border cell a side)
    quiet zone  140 px  each side
    total      1120 px

That 0.75 ratio is what ties this image to the geometry in model.sdf. The
model's plate is 0.192 m square so that the MARKER ITSELF is 0.144 m --
the number that goes into pose estimation later. Change one, change the other.
"""

from pathlib import Path

import cv2
import numpy as np

DICTIONARY = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
CELL_PX = 140          # one cell of the marker grid
GRID_CELLS = 6         # 4 data cells + 1 black border cell each side
QUIET_CELLS = 1        # white margin, in cells

MARKER_PX = CELL_PX * GRID_CELLS
PAD_PX = CELL_PX * QUIET_CELLS

OUT_DIR = Path(__file__).resolve().parent.parent / "models/aruco_marker/materials/textures"


def main() -> None:
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    marker = cv2.aruco.generateImageMarker(aruco_dict, MARKER_ID, MARKER_PX)

    total = MARKER_PX + 2 * PAD_PX
    canvas = np.full((total, total), 255, dtype=np.uint8)
    canvas[PAD_PX:PAD_PX + MARKER_PX, PAD_PX:PAD_PX + MARKER_PX] = marker

    # Write RGB, not greyscale. Ogre2 (Gazebo's renderer) will happily load a
    # single-channel PNG as an albedo map and then render it pure black, with
    # no error logged. Three identical channels look the same and work.
    canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / f"aruco_4x4_id{MARKER_ID}.png"
    cv2.imwrite(str(out_path), canvas)

    print(f"wrote {out_path}")
    print(f"  image      {total}x{total} px")
    print(f"  marker     {MARKER_PX} px  ({MARKER_PX / total:.2%} of width)")
    print(f"  dictionary DICT_4X4_50, id {MARKER_ID}")


if __name__ == "__main__":
    main()
