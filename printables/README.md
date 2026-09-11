# printables

Two A3 sheets. One is the tag the drone tracks, the other is the target the
camera is calibrated against.

| File | What it is |
|---|---|
| `aruco_4x4_id0_200mm.pdf` | DICT_4X4_50 id 0. Black marker 200 mm square, 266.7 mm including the white quiet zone |
| `charuco_7x10_35mm.pdf` | ChArUco 7x10, DICT_5X5_100, 35 mm squares, 26.25 mm markers, 245 x 350 mm board |
| `make_printables.py` | Regenerates both. Host-side, `uv run printables/make_printables.py` from `corvid/` |

## Printing

**100%, or "actual size". Never "fit to page".** That single setting is the
difference between a tag that measures right and one that lies to you by a few
percent forever.

**Laser, not inkjet, and matte, not glossy.** Gloss throws specular highlights
under lab lights and washes out the black cells, which is a detection failure
rather than an accuracy one.

## After printing, measure the ruler

Both sheets carry a 100 mm ruler. It is there because nothing in software can
tell you what the printer actually did. Measure it. If it reads 100 mm you are
fine; if it reads 99 mm the sheet came out 1% small and everything below applies.

The two sheets then care about scale completely differently:

**The board barely cares.** Camera intrinsics come out of the ratios between
corners, so a uniformly scaled board gives identical `fx`, `fy`, `cx`, `cy` and
distortion coefficients. Only the board's estimated distance shifts, and we throw
that away. Measure a square with calipers and feed the measured number in anyway
-- it costs nothing and it removes a doubt.

**The tag cares directly.** `solvePnP` takes the marker side in metres, so a tag
printed 1% small reports every range 1% short. At a 2 m hold height that is 2 cm,
which is the entire error budget spent on a print dialog. Measure the black
square edge to edge and put the MEASURED value into `marker_size_m`, rather than
assuming 0.200.

## Mounting

**Flat matters more than anything else here**, and it matters most for the board:
a bow gets absorbed into the distortion coefficients, where it silently poisons
every calibration done afterwards. Glass with the sheet adhered to it is the
flattest surface that is easy to get. Foam board is the practical second choice
and is what the tag is going on anyway -- just know that foam board bends over
time, so check it again before it matters.

**Do not trim into the white.** The quiet zone around the tag is not a margin,
it is part of the detector's job -- without it the tag is not found at all. Same
for the board.

## Why these choices

**ChArUco rather than a plain checkerboard.** A checkerboard has to be fully
visible in every image, so it can never be pushed into the corners of the frame
-- and the corners are where lens distortion lives and where its coefficients are
actually determined. Every white square here carries a coded marker, so partial
views still count.

**7 x 10 -- one dimension odd, one even.** A board with both dimensions odd or
both even is identical to its own 180-degree rotation, and the calibration has no
way to tell which way round it was.

**The board uses DICT_5X5_100, the tag uses DICT_4X4_50.** If both came from one
dictionary the board's 35 coded squares would include id 0, and a calibration
board lying in shot during a flight test would publish detections the station
keeper would fly towards. Different dictionaries make that impossible instead of
unlikely.

**The patterns are rendered by OpenCV itself, not drawn by us.** OpenCV ships its
own SVG pattern generator that re-implements the ChArUco layout, and that
re-implementation is where opencv#23152 came from: 4.6 changed the layout, boards
printed under one version stopped detecting under another, and nothing warned
anybody. Here the library renders the board and the same library detects it, so
they cannot disagree. `make_printables.py` pins `opencv-contrib-python==4.10.0.84`
to match the container, and re-detects its own render on every run -- a mismatch
fails on your laptop instead of on a printed sheet in the lab.

## What was verified, and how

Not inferred -- the PDFs were rasterised back to images and the real detectors
were run on them:

- board: 54 of 54 interior corners found, square pitch 35.014 mm measured against
  35.000 nominal
- tag: detected as id 0 under `CORNER_REFINE_APRILTAG`, all four sides 200.15 mm
  against 200.000 nominal
- a board rendered under OpenCV 4.10 detects cleanly under 4.13

The residual tenths of a millimetre are the measuring raster at 200 dpi, where one
pixel is 0.127 mm. They are not in the PDF.
