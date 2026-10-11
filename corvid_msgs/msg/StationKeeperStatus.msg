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

# Where the marker is, in metres, forward and left of the drone. This is the
# controller's error signal.
#
# The frame is the LEVEL body frame: the drone's own heading, with roll and
# pitch taken out using the attitude below. It is not the raw optical-to-body
# result. A tilted camera projects a marker on the ground to a false offset of
# about height x tan(tilt) -- measured at 19 cm at 3-5 degrees from 2.3 m --
# and removing the tilt is what makes these numbers mean what they say.
float32 offset_forward_m
float32 offset_left_m

# The attitude that was taken out, degrees, from the sample nearest the source
# image's own timestamp -- NOT the newest attitude at the time of the tick.
# With around 0.18 s of pipeline latency and tilt rates near 10 deg/s, the
# newest attitude is not the attitude the picture was taken at.
#
# NaN if no attitude was available for that instant, in which case the offsets
# above are raw, commanding is false, and nothing was flown on them.
float32 roll_deg
float32 pitch_deg

# Source image stamp minus the attitude sample's stamp, seconds. How well the
# two were actually matched, rather than how well they were assumed to be.
# Positive means the attitude sample is older than the image. NaN if none.
float32 attitude_age_s

# Latency compensation. The offsets above are what the camera saw at the
# instant the picture was taken; by the time a tick acts on them the drone has
# moved. When compensated is true, PX4's own record of how far it moved since
# that picture was subtracted, and comp_*_m is exactly what that changed, in the
# body frame as it is NOW. The controller's error was offset + comp.
#
# compensated is false when the feature is switched off, when no PX4 position
# was within 50 ms of the image, when x/y were invalid, or when PX4 reset its
# position estimate in between - and then comp_*_m is NaN and the raw offset
# was flown, exactly as before this existed.
bool compensated
float32 comp_forward_m
float32 comp_left_m

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
