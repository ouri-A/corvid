# What the station keeper decided, and what it decided it from.
#
# Purely diagnostic. Nothing subscribes to this in flight and nothing should --
# it exists so a recorded flight can be read back as what the controller
# ACTUALLY did, rather than reconstructed afterwards from the marker pose, the
# vehicle heading and the gains. Those reconstructions were how the first
# flight got analysed and they can silently disagree with the node itself,
# which is the one thing a post-mortem must never do.
#
# Published once per setpoint tick, at the same rate as the setpoint, whether
# or not a marker was seen.

# stamp    - when this decision was made, on the node's clock (sim time under
#            use_sim_time). NOT the detection's stamp; that one is recoverable
#            as stamp minus marker_age_s.
# frame_id - the body frame the offsets and body-frame velocities below are in.
std_msgs/Header header

# Which marker this node is locked to. Echoed so a bag is self-describing and
# a Stage 3 recording with several drones can be told apart.
uint32 target_marker_id

# True if the last detection message contained this marker.
bool marker_visible

# True if no detection of it has arrived within the timeout, so the marker-lost
# branch ran on this tick. This is the flag that gates behaviour -- read this
# rather than inferring it from marker_age_s and a timeout you assume.
bool marker_lost

# True if this tick produced a real lateral command: a marker in time AND a
# vehicle heading to rotate it by. False means the velocities below are zeros
# by default rather than by control.
bool commanding

# Seconds since the source image of the newest accepted detection. NaN if no
# marker has ever been seen. Includes perception latency, so it does not go to
# zero even when tracking perfectly -- around 0.1 s is normal.
float32 marker_age_s

# Where the marker is, in the body frame, in metres. Forward and left, from the
# hardcoded optical-to-body rotation. This is the controller's error signal.
float32 offset_forward_m
float32 offset_left_m

# The proportional and integral contributions, separately, in m/s. Split so a
# flight flown with a nonzero ki can be read without re-deriving which part of
# the command came from where. Both are post-clamp for the integral term and
# pre-clamp for the total.
float32 p_forward_mps
float32 p_left_mps
float32 i_forward_mps
float32 i_left_mps

# The lateral command in the body frame after the speed clamp, in m/s.
float32 cmd_forward_mps
float32 cmd_left_mps

# The same command after rotation into local NED -- what actually went into the
# TrajectorySetpoint. Recording both halves is what makes a frame error
# separable from a control error.
float32 cmd_north_mps
float32 cmd_east_mps

# The vehicle yaw used for that rotation, radians. NaN if none had arrived.
float32 heading_rad

# Altitude. z_m is the EKF's current vertical position and
# altitude_setpoint_m is what was commanded, both NED metres, so both are
# NEGATIVE above the takeoff point.
float32 z_m
float32 altitude_setpoint_m
