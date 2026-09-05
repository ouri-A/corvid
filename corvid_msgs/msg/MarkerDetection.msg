# One detected ArUco marker.
#
# No header here on purpose: the timestamp and frame belong to the camera
# image, and every detection from a single image shares them. They live on
# MarkerDetectionArray instead.

# Decoded marker ID. DICT_4X4_50 spans 0-49; uint32 is the convention
# aruco_ros uses and leaves room for a larger dictionary later.
uint32 id

# Marker pose in the camera optical frame, from solvePnP.
# Metres and quaternion. The frame is named in the array's header.
geometry_msgs/Pose pose
