# Every ArUco marker found in one camera image.
#
# Published once per processed frame, whether or not anything was found.
# An EMPTY detections array is meaningful: it says the camera looked at
# this instant and saw nothing. That is different from no message arriving
# at all, which means the pipeline has stalled. A subscriber must treat
# those two cases differently or it will trigger marker-lost behaviour on
# a slow frame -- camera intervals here range from 20 ms to 131 ms.

# stamp    - acquisition time of the source image, copied from it.
#            Consumers use it to reject poses that are too old to act on.
# frame_id - the camera optical frame the poses below are expressed in.
std_msgs/Header header

MarkerDetection[] detections
