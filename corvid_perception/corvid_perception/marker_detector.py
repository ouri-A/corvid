"""Corvid's marker detector.

Detects ArUco markers in the drone's downward camera and publishes their metric
6-DoF pose. That is what this node is for; what follows is how much of it has
been built.

Implemented so far: acquiring the camera intrinsics from camera_info and
rejecting them if they are unusable, detecting ArUco markers in the image
stream, recovering each marker's metric pose with solvePnP, resolving the
planar-marker pose ambiguity that solver leaves behind, and publishing them as
a MarkerDetectionArray.

Detection needs no intrinsics - only pose does - so those two halves are
deliberately independent: without usable camera_info this node degrades to a
detector that logs what it sees. It publishes nothing in that state, because the
topic's contract is poses in the optical frame and it cannot honour that. The
topic falling silent is the correct signal that the pipeline is not functional.

Poses are still logged as well as published. The log is what was used to
cross-check the reported distance against PX4's own dist_bottom, and it costs
almost nothing.

There is also an annotated debug image, drawn and published only while
something is subscribed to it. It exists to be looked at; nothing consumes it.

Still to come: broadcasting TF.

Topic names here are short and relative on purpose. They are remapped at launch,
so this file is identical in simulation and on the real drone.
"""

import math

import cv2
import numpy as np
import rclpy
from corvid_msgs.msg import MarkerDetection, MarkerDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def quaternion_from_rotation_matrix(m):
    """Convert a 3x3 rotation matrix to an (x, y, z, w) quaternion.

    solvePnP returns rotation as a Rodrigues vector - an axis scaled by the
    angle - and geometry_msgs/Pose wants a quaternion, so a conversion is
    unavoidable. cv2.Rodrigues covers the first half of it; this covers the
    second.

    The four branches are not four different formulas. Each one divides by a
    term that goes to zero for some rotation, so the branch with the largest
    such term is chosen and the others avoided. Taking only the first would
    lose precision near a 180-degree rotation and divide by zero at it.
    """
    trace = m[0, 0] + m[1, 1] + m[2, 2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        )

    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (
            0.25 * s,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[2, 1] - m[1, 2]) / s,
        )

    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return (
            (m[0, 1] + m[1, 0]) / s,
            0.25 * s,
            (m[1, 2] + m[2, 1]) / s,
            (m[0, 2] - m[2, 0]) / s,
        )

    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return (
        (m[0, 2] + m[2, 0]) / s,
        (m[1, 2] + m[2, 1]) / s,
        0.25 * s,
        (m[1, 0] - m[0, 1]) / s,
    )


def marker_tilt_deg(rotation):
    """How far the marker is turned away from facing the camera squarely.

    The marker's own +z axis is its normal, pointing up out of the ground
    towards the drone. Rotating that into the camera optical frame is exactly
    the third column of the rotation matrix, so rotation[2, 2] is the component
    of the marker's normal along the camera's viewing direction.

    The optical frame's z points away from the lens into the scene, so a marker
    lying flat beneath a level drone has a normal reading close to (0, 0, -1)
    and rotation[2, 2] close to -1. Zero degrees means facing the camera dead
    on; ninety means edge on.

    The clamp is against floating-point drift pushing the argument a hair past
    -1 or 1, where acos raises instead of returning 0 or pi.
    """
    return math.degrees(math.acos(max(-1.0, min(1.0, -rotation[2, 2]))))


class MarkerDetector(Node):
    """Reports which markers are in frame, and where they are in metres."""

    def __init__(self):
        super().__init__('marker_detector')

        # None until a camera_info message arrives and passes validation.
        self._camera_info = None

        # A parameter rather than a constant because the printed marker in
        # Stage 2 may not be exactly this size, and that difference belongs in
        # a launch file, not in an edit to this source on the drone.
        self.declare_parameter('marker_size_m', 0.144)
        self._marker_size_m = (
            self.get_parameter('marker_size_m').get_parameter_value().double_value
        )

        # The marker's own four corners, in its own frame, in metres. Built
        # once. Order matches what detectMarkers returns - top-left,
        # top-right, bottom-right, bottom-left - because solvePnP pairs object
        # points with image points positionally, and a mismatched order yields
        # a pose that looks perfectly reasonable and is rotated by 90 degrees.
        # z is zero for all four: the marker is planar, and this is the plane.
        half = self._marker_size_m / 2.0
        self._object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )

        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera_info',
            self._on_camera_info,
            10,
        )

        self._bridge = CvBridge()

        # Built once, not per frame: the dictionary holds the precomputed bit
        # patterns for all 50 markers.
        #
        # Corner refinement is the one detector parameter that matters here,
        # and it matters far more than its name suggests. Range to the marker
        # is depth from apparent size - roughly focal length times marker size
        # over the marker's width in pixels - so an error in that pixel width
        # is an error in range, proportionally. The marker spans around 39 px
        # at hover, so one pixel is about 7 cm of range and one marker module,
        # six and a half pixels, is about 45 cm. Corner placement is therefore
        # the whole range error budget, and which refinement method places them
        # is not a detail.
        #
        # APRILTAG, and the reasoning is worth keeping because it was expensive.
        #
        # Measured on 371 recorded frames of a real hover, scored against the
        # rangefinder, on the OpenCV this image actually runs (4.10):
        #
        #     SUBPIX     stdev 19.2 cm    85 of 371 frames badly wrong
        #     NONE       stdev 19.9 cm    85
        #     CONTOUR    stdev 31.4 cm    87
        #     APRILTAG   stdev  5.9 cm     0
        #
        # The first three put the corners a whole marker module too far out on
        # roughly a quarter of frames, which reads as the marker being 15%
        # closer than it is - around 35 cm at this range. It is not noise: the
        # bad frames are as internally consistent as the good ones, which is
        # why it survived so long disguised as a bimodal "camera offset".
        #
        # APRILTAG does not refine each corner independently. It fits lines to
        # the marker's four edges and intersects them, so a corner cannot
        # wander outward on its own - it is pinned by two fitted edges. That is
        # the property that matters here, and it is why this is not a
        # workaround for one OpenCV version: APRILTAG is also the best of the
        # four under OpenCV 5, where the others behave themselves.
        #
        # It costs about 1% of detections, being slightly stricter: 366 frames
        # instead of 371. Five lost against eighty-five badly wrong.
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector_params = cv2.aruco.DetectorParameters()
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

        # Best effort, keep last, DEPTH 1. Best effort because a real camera
        # driver publishes best effort, and a best-effort publisher cannot feed
        # a reliable subscriber - so this works against both sim and hardware.
        #
        # Depth 1 is the important part. The stock qos_profile_sensor_data is
        # KEEP_LAST with depth FIVE (rmw qos_profiles.h, Humble). While the
        # detector keeps up with the camera that never matters; once it falls
        # behind - the Orin manages ~6 Hz against a 30 Hz camera - up to five
        # frames queue and every detection is made on a stale one. Measured
        # 5 Oct on the Orin: detections already 0.37-0.49 s old on arrival,
        # against 0.1 s on the laptop, enough to trip marker-lost constantly.
        # Depth 1 drops the backlog: the next frame processed is always the
        # newest.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_sub = self.create_subscription(
            Image,
            'image_raw',
            self._on_image,
            image_qos,
        )

        # Depth 1 and, by taking the plain int form, default reliable QoS.
        # The compatibility argument runs the opposite way for a publisher than
        # it does for a subscriber: a best-effort publisher cannot feed a
        # reliable subscriber, so publishing reliably is what works against
        # any subscriber anyone writes later. Depth 1 keeps that from meaning
        # "retransmit stale poses" - the newest is the only one worth having.
        self._detections_pub = self.create_publisher(
            MarkerDetectionArray, 'marker_detections', 1
        )

        # The annotated frame, for looking at rather than acting on. Nothing
        # in the pipeline consumes it.
        #
        # It is drawn and published only while something is subscribed, which
        # is checked per frame. A 1280x960 colour image at 13 Hz is around
        # 48 MB/s and the drawing costs CPU on every frame, so left ungated
        # this would tax every flight for the sake of the rare one where
        # somebody is watching. Gated, it costs nothing until an image viewer
        # connects, and turns itself on the moment one does.
        self._debug_image_pub = self.create_publisher(Image, 'debug_image', 1)

        # Turns "nothing is happening" into an explicit complaint. Cancelled
        # as soon as intrinsics arrive.
        self._waiting_timer = self.create_timer(5.0, self._warn_if_waiting)

        # On the record in every run. This one number sets the scale of every
        # pose the node reports, and a wrong value fails silently - the poses
        # stay plausible and are uniformly wrong.
        self.get_logger().info(
            'marker size %.4f m' % self._marker_size_m
        )

        self.get_logger().info(
            "waiting for intrinsics on '%s'" % self._camera_info_sub.topic_name
        )

    def _on_camera_info(self, msg):
        # Already have good intrinsics: keep the latest and say nothing.
        if self._camera_info is not None:
            self._camera_info = msg
            return

        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]

        if fx == 0.0 or fy == 0.0:
            self.get_logger().error(
                "camera_info on '%s' has a zero focal length. This is what an "
                'uncalibrated camera publishes. solvePnP would accept it and '
                'return meaningless poses, so these intrinsics are being '
                'rejected.' % self._camera_info_sub.topic_name,
                throttle_duration_sec=5.0,
            )
            return

        self._camera_info = msg
        self._waiting_timer.cancel()

        self.get_logger().info(
            "intrinsics acquired from '%s': %dx%d, fx=%.2f fy=%.2f "
            'cx=%.2f cy=%.2f, distortion_model=%s, optical frame=%s'
            % (
                self._camera_info_sub.topic_name,
                msg.width,
                msg.height,
                fx,
                fy,
                cx,
                cy,
                msg.distortion_model,
                msg.header.frame_id,
            )
        )

    def _on_image(self, msg):
        # desired_encoding does the colour conversion for us, so the node does
        # not care whether the driver hands it mono, BGR or Bayer. That is a
        # real sim-to-hardware difference, handled in one string.
        try:
            gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError as exc:
            self.get_logger().error(
                'could not convert image to mono8: %s' % exc,
                throttle_duration_sec=5.0,
            )
            return

        # The third return value is the rejected candidates - a debugging aid
        # for "the marker is visible but not decoding". Not needed yet.
        corners, ids, _rejected = self._detector.detectMarkers(gray)

        # Throttled because at ~24 Hz an unthrottled log line is unreadable.
        if ids is None:
            self.get_logger().info(
                'no markers in frame', throttle_duration_sec=2.0
            )
        else:
            self.get_logger().info(
                'markers seen: %s' % sorted(int(i) for i in ids.flatten()),
                throttle_duration_sec=2.0,
            )

        # Pose cannot run without intrinsics, and neither can publishing: the
        # message promises metric poses in a named frame, and without
        # camera_info there is neither a metric pose nor a frame to name.
        if self._camera_info is None:
            self.get_logger().warn(
                'no usable camera_info yet, so nothing is being published',
                throttle_duration_sec=5.0,
            )
            return

        detections = MarkerDetectionArray()

        # The image's own acquisition time, not now. Copying it is what lets a
        # consumer measure how old a pose is before acting on it - one of the
        # numbers Objective 3 needs instrumented from the start.
        detections.header.stamp = msg.header.stamp

        # The optical frame the poses are expressed in, taken from camera_info
        # rather than from the image so the frame and the intrinsics that
        # produced the poses can never disagree with each other.
        detections.header.frame_id = self._camera_info.header.frame_id

        # What the debug image needs that the detections do not: the corners as
        # found, the corners as reprojected, and the numbers printed alongside.
        annotations = []

        if ids is not None:
            camera_matrix = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
            dist_coeffs = np.array(self._camera_info.d, dtype=np.float64)

            for marker_id, marker_corners in zip(ids.flatten(), corners):
                detection, annotation = self._detection_for(
                    int(marker_id), marker_corners, camera_matrix, dist_coeffs
                )
                if detection is not None:
                    detections.detections.append(detection)
                    annotations.append(annotation)

        # Published on every processed frame, empty array included. An empty
        # array is a timestamped "looked, saw nothing", which a subscriber must
        # not confuse with no message arriving at all.
        self._detections_pub.publish(detections)

        if self._debug_image_pub.get_subscription_count() > 0:
            self._publish_debug_image(msg, gray, annotations)

    def _detection_for(self, marker_id, marker_corners, camera_matrix, dist_coeffs):
        # detectMarkers hands back (1, 4, 2); solvePnP wants (4, 2).
        image_points = marker_corners.reshape(4, 2).astype(np.float64)

        # IPPE_SQUARE is the planar-square solver, and the ONE thing to
        # understand here is that it does not remove the ambiguity - it exposes
        # it. Four coplanar points admit two poses that reproject almost
        # identically, roughly a flip of the marker about a plane through the
        # line of sight, and no PnP algorithm can choose between them because
        # the information is not in the corners. It is worst when the
        # projection is near affine: small, distant, or viewed head on. A
        # downward camera hovering over a ground marker is all three.
        #
        # solvePnPGeneric returns both solutions where solvePnP silently
        # returned one, which was wrong on roughly half the frames and showed
        # up as a z error with two modes 25-30 cm apart.
        _n_solutions, rvecs, tvecs, _errors = cv2.solvePnPGeneric(
            self._object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not rvecs:
            self.get_logger().warn(
                'solvePnPGeneric returned no solution for marker %d' % marker_id,
                throttle_duration_sec=2.0,
            )
            return None, None

        # Both candidates are graded the same way the single pose used to be:
        # push the marker's corners back through the recovered pose and measure
        # how far they land from the corners the detector actually found. This
        # is recomputed here rather than taken from solvePnPGeneric's own error
        # output so the number stays directly comparable with the 0.23-0.70 px
        # baseline already measured with this formula.
        candidates = []
        for rvec, tvec in zip(rvecs, tvecs):
            projected, _jacobian = cv2.projectPoints(
                self._object_points, rvec, tvec, camera_matrix, dist_coeffs
            )
            projected = projected.reshape(4, 2)
            residuals = projected - image_points
            error_px = float(np.sqrt((residuals**2).sum(axis=1).mean()))
            rotation, _jacobian = cv2.Rodrigues(rvec)
            candidates.append(
                (rvec, tvec, rotation, error_px, marker_tilt_deg(rotation), projected)
            )

        # The prior that breaks the tie. Reprojection error cannot do it - in
        # this geometry the two errors are near enough equal that corner noise
        # decides, which is the whole problem - so the choice is made with
        # information from outside the image: the marker lies flat on the
        # ground and the camera looks roughly down at it, so of the two poses
        # the one tilted less far from facing the camera is the real one.
        #
        # This is deliberately the cheapest form of that prior, a constant
        # standing in for the drone's true attitude. It is exact in a level
        # hover and weakens as the drone tilts - but the ambiguity itself is
        # worst at nadir and weakens as the drone tilts too, because the
        # projection stops being affine and the two reprojection errors
        # separate. The two effects cover each other. Replacing the constant
        # with PX4's gravity-referenced attitude is the upgrade, and needs the
        # camera-to-body transform first.
        #
        # The assumption being made, written down because it is invisible in
        # the arithmetic: the marker is horizontal. True on the floor and on a
        # ground robot's deck, false on a wall.
        chosen = min(candidates, key=lambda c: c[4])
        rvec, tvec, rotation, reprojection_error_px, tilt_deg, projected = chosen

        x, y, z = (float(v) for v in tvec.flatten())

        # Mean length of the four sides of the detected quadrilateral, in
        # pixels. This is the number range is computed from - range is roughly
        # focal length times marker size over this - so it is the single most
        # diagnostic quantity the node has, and it is not recoverable from the
        # published pose alone. Two frames that disagree about range disagree
        # about this and nothing else.
        edges = np.roll(image_points, -1, axis=0) - image_points
        edge_px = float(np.sqrt((edges**2).sum(axis=1)).mean())

        # Every candidate is logged, not just the winner. Whether this prior
        # actually works is decided by how far apart the two tilts are: a wide
        # margin means the choice is not close, near-equal tilts mean it is a
        # coin flip and the prior is not doing its job.
        self.get_logger().info(
            'marker %d: x=%+.3f y=%+.3f z=%+.3f m (camera optical frame), '
            'edge %.1f px, reprojection error %.2f px, tilt %.1f deg '
            '| candidates %s'
            % (
                marker_id,
                x,
                y,
                z,
                edge_px,
                reprojection_error_px,
                tilt_deg,
                ', '.join(
                    '%.2f px @ %.1f deg' % (c[3], c[4]) for c in candidates
                ),
            ),
            throttle_duration_sec=2.0,
        )

        detection = MarkerDetection()
        detection.id = marker_id
        detection.pose.position.x = x
        detection.pose.position.y = y
        detection.pose.position.z = z

        # The rotation matrix already exists - it was built above to measure
        # the tilt - so all that is left is the conversion to the quaternion
        # geometry_msgs wants.
        qx, qy, qz, qw = quaternion_from_rotation_matrix(rotation)
        detection.pose.orientation.x = qx
        detection.pose.orientation.y = qy
        detection.pose.orientation.z = qz
        detection.pose.orientation.w = qw

        annotation = {
            'id': marker_id,
            'detected': image_points,
            'projected': projected,
            'range_m': math.sqrt(x * x + y * y + z * z),
            'z_m': z,
            'edge_px': edge_px,
            'error_px': reprojection_error_px,
            'tilt_deg': tilt_deg,
            'candidates': [(c[3], c[4]) for c in candidates],
        }

        return detection, annotation

    # Drawing colours, BGR. Green is what the detector found, red is what the
    # pose we chose predicts. Where they disagree, the pose is wrong; where
    # they agree but sit off the marker's real edge, the corners are.
    DETECTED_COLOUR = (0, 255, 0)
    PROJECTED_COLOUR = (0, 0, 255)
    TEXT_COLOUR = (0, 255, 255)

    # Widest the magnified inset is allowed to get, in output pixels, on a
    # 1280-wide frame. Scaled with the frame width, so it covers the same share
    # of the picture on any camera (200 px on the VX-500's 640x480).
    INSET_MAX_PX = 400

    def _publish_debug_image(self, msg, gray, annotations):
        """Publish the frame with the detection drawn on it.

        Only ever called when something is subscribed.

        The inset is the point of this, not a nicety. The marker is around 38
        pixels across in a 1280x960 frame, and the errors worth seeing are on
        the order of one marker module - about 6 pixels. At full size that is
        a smudge. Magnified with nearest-neighbour, the source pixel grid stays
        visible, so it is possible to see which edge in the image the corner
        has actually landed on.
        """
        canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        height, width = gray.shape[:2]

        for ann in annotations:
            detected = ann['detected'].astype(np.int32)
            cv2.polylines(canvas, [detected], True, self.DETECTED_COLOUR, 1)
            for (u, v) in ann['projected']:
                cv2.circle(canvas, (int(round(u)), int(round(v))), 3,
                           self.PROJECTED_COLOUR, 1)

        if annotations:
            # The inset follows the first marker. With one marker in frame that
            # is the marker; with several it is an arbitrary but stable choice,
            # and the full-size drawing still covers the rest.
            ann = annotations[0]
            pts = ann['detected']
            pad = 8
            x0 = max(0, int(pts[:, 0].min()) - pad)
            y0 = max(0, int(pts[:, 1].min()) - pad)
            x1 = min(width, int(pts[:, 0].max()) + pad + 1)
            y1 = min(height, int(pts[:, 1].max()) + pad + 1)

            if x1 - x0 >= 2 and y1 - y0 >= 2:
                # Integer zoom, so every source pixel becomes an exact square
                # block and the grid reads cleanly.
                inset_max = self.INSET_MAX_PX * width // 1280
                zoom = max(1, min(8, inset_max // max(x1 - x0, y1 - y0)))

                # Cropped from the CLEAN image, not from the annotated one -
                # upscaling lines already drawn at full size would thicken them
                # into exactly the few pixels being inspected.
                crop = cv2.cvtColor(gray[y0:y1, x0:x1], cv2.COLOR_GRAY2BGR)
                inset = cv2.resize(crop, None, fx=zoom, fy=zoom,
                                   interpolation=cv2.INTER_NEAREST)

                # Subpixel coordinates scale straight through: a corner at
                # u = 10.3 in the source sits at 10.3 * zoom in the inset, and
                # whole-number source coordinates land on block boundaries.
                for (u, v) in ann['detected']:
                    cv2.drawMarker(inset, (int(round((u - x0) * zoom)),
                                           int(round((v - y0) * zoom))),
                                   self.DETECTED_COLOUR, cv2.MARKER_CROSS, 11, 1)
                for (u, v) in ann['projected']:
                    cv2.circle(inset, (int(round((u - x0) * zoom)),
                                       int(round((v - y0) * zoom))),
                               3, self.PROJECTED_COLOUR, 1)

                ih, iw = inset.shape[:2]
                if ih <= height and iw <= width:
                    canvas[0:ih, 0:iw] = inset
                    cv2.rectangle(canvas, (0, 0), (iw - 1, ih - 1),
                                  self.TEXT_COLOUR, 1)
                    cv2.putText(canvas, 'x%d' % zoom, (4, ih - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                self.TEXT_COLOUR, 1, cv2.LINE_AA)

        if annotations:
            ann = annotations[0]
            lines = [
                'id %d   range %.3f m   z %.3f m'
                % (ann['id'], ann['range_m'], ann['z_m']),
                # A 4x4 marker is six modules across counting its border. The
                # module size is printed because the error being hunted is a
                # whole module wide, and comparing 37.9 against 44.2 px is
                # easier as 6.3 against 7.4.
                'edge %.1f px   module %.2f px'
                % (ann['edge_px'], ann['edge_px'] / 6.0),
                'reproj %.2f px   tilt %.1f deg' % (ann['error_px'], ann['tilt_deg']),
                'candidates ' + '  '.join(
                    '%.2fpx@%.1fdeg' % (e, t) for e, t in ann['candidates']
                ),
            ]
        else:
            lines = ['no marker detected']

        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (width - 430, 24 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.TEXT_COLOUR, 1,
                        cv2.LINE_AA)

        try:
            out = self._bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(
                'could not build the debug image: %s' % exc,
                throttle_duration_sec=5.0,
            )
            return

        # Same stamp and frame as the detections, so a viewer showing this
        # beside them is showing the same instant.
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._camera_info.header.frame_id
        self._debug_image_pub.publish(out)

    def _warn_if_waiting(self):
        self.get_logger().warn(
            "still no usable camera_info on '%s' after 5s - check the remap, "
            'and that the bridge is running'
            % self._camera_info_sub.topic_name
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
