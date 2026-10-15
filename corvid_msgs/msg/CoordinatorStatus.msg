# What the swarm coordinator saw, decided and sent, for one bot.
#
# Purely diagnostic, like StationKeeperStatus: nothing subscribes to it to
# act. It exists so a bag can be read back as what the coordinator ACTUALLY
# did, and so a later viz node can turn it into RViz markers without
# recomputing anything.
#
# Published on every detection frame the coordinator takes in, whether or not
# a run is going, and at least twice a second if frames stop arriving.

# stamp    - acquisition time of the camera image these numbers came from (the
#            detection the coordinator acted on, after any extra_delay_s). If
#            no frame has arrived recently, the node's own time instead.
# frame_id - the camera optical frame. Every position and pose below is in it,
#            in metres, so RViz can show them with no TF tree.
std_msgs/Header header

uint8 PHASE_DISABLED=0          # no run; watching only, sending nothing
uint8 PHASE_MEASURING=1         # bot stopped, waiting for a settled frame to decide from
uint8 PHASE_TURNING=2           # turn sent, waiting for the bot to report idle
uint8 PHASE_DRIVING=3           # drive being streamed, camera deciding when to stop
uint8 PHASE_WAITING_FOR_TAGS=4  # a tag went missing, bot stopped, waiting for it
uint8 PHASE_STOPPING=5          # stop sent, waiting for the bot to report idle
uint8 PHASE_ARRIVED=6           # run over: within tolerance of the goal
uint8 PHASE_GAVE_UP=7           # run over: out of rounds, or something failed
uint8 phase

# 1-based round within the current run. 0 when no run has started.
uint8 round

uint32 bot_marker_id
uint32 target_marker_id

# Whether each tag was in the frame these numbers came from.
bool bot_visible
bool target_visible

# Seconds between the image's capture and the moment the coordinator used it.
# This is the age the stop point is computed from. NaN if no frame.
float32 detection_age_s

# The small-tag correction. The detector solves every tag at one size; tags
# whose real size differs are scaled by this. 1.0 for a tag of the size the
# detector assumed.
float32 bot_tag_scale
float32 target_tag_scale

# bot_pose: the bot's PIVOT (midpoint of the drive wheels), with orientation
# as the bot's own body frame - x forward, y left, z up - so an RViz arrow
# points the way the bot faces. Built from the tag pose, the tag's mounting
# (top edge = front) and tag_offset. Scaled. Invalid (all NaN) if not seen.
geometry_msgs/Pose bot_pose

# Centre of the target tag. Scaled. NaN if not seen.
geometry_msgs/Point target

# The goal point on the ring round the target, in the bot's plane. NaN if no
# run, or if the target was not seen.
geometry_msgs/Point goal

# From the bot's pivot to the goal, measured in the bot's own plane:
# distance, and the angle from the bot's forward axis, + = to its left.
float32 distance_to_goal_m
float32 bearing_to_goal_rad

# Distance left along the bot's heading, and the stopping lead it was compared
# with (drive_speed x detection_age_s). Driving stops when remaining <= lead.
# NaN outside DRIVING.
float32 drive_remaining_m
float32 stop_lead_m

# The last turn sent in this run (rad, + = left) and the last drive value
# sent (m/s; 0.0 is the stop command). NaN if none sent yet.
float32 sent_turn_rad
float32 sent_drive_mps

# /botN/state as last heard (0 idle, 1 turning, 2 driving), and how long ago
# it arrived. 255 and NaN if never heard.
uint8 bot_state
float32 bot_state_age_s

# Why the current phase was entered, in words. At the end of a run this is the
# outcome, e.g. "arrived, 3.1 cm from goal" or "gave up after 5 rounds, 7.2 cm".
string reason
