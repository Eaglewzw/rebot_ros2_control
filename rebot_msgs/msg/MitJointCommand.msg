# Full Damiao MIT-mode command tuple for a set of joints.
# Semantics map 1:1 onto the MIT control frame:
#   tau_motor = kp * (position - q) + kd * (velocity - dq) + effort
#
# All arrays must either be empty or have the same length as joint_names.
# An empty array (or a NaN element) means "not set": the hardware then falls
# back to its safe default (hold position / zero velocity & effort / URDF
# default kp & kd).

std_msgs/Header header

string[] joint_names

# Desired position [rad or m, joint space]
float64[] position
# Desired velocity [rad/s or m/s]
float64[] velocity
# MIT proportional gain, range [0, 500]
float64[] kp
# MIT derivative gain, range [0, 5]
float64[] kd
# Feed-forward torque/force [N*m or N, joint space]
float64[] effort
