'''
Robotiq 2F-140 fingertip pad gap for the gripper pre-open check. Table from the real robot's URDF (sampled every
0.05 rad, linear in between, gap within 0.6 mm): pad gap 139.5 mm open at 0 rad, closed at 0.755 rad.
Pure functions, no ROS.
'''

THETA_STEP = 0.05
# pad gap (m) at theta = 0, 0.05, ... 0.75 rad
PAD_GAP_M = [0.1395, 0.1317, 0.1237, 0.1153, 0.1066, 0.0978, 0.0887, 0.0794, 0.0699, 0.0603, 0.0505, 0.0407,
             0.0308, 0.0208, 0.0108, 0.0008]
MAX_THETA = 0.755
PRE_OPEN_GAP_TOLERANCE_M = 0.015   # the open fingers read 0.048 rad = 132 mm of the configured 140 mm (sim)


def pad_gap(theta):
    '''gap (m) between the two pad faces at finger angle theta (rad), clamped to the stroke'''
    theta = min(max(theta, 0.0), MAX_THETA)
    index = min(int(theta / THETA_STEP), len(PAD_GAP_M) - 2)
    fraction = min((theta - index * THETA_STEP) / THETA_STEP, 1.0)
    return PAD_GAP_M[index] + (PAD_GAP_M[index + 1] - PAD_GAP_M[index]) * fraction


def reads_open(theta, open_width, tolerance=PRE_OPEN_GAP_TOLERANCE_M):
    '''True when the finger angle theta (rad) puts the pads within tolerance (m) of the configured open width (m), so a
    pre-open goal would change nothing; False for a missing reading (None) or a partly closed gripper'''
    if theta is None:
        return False
    return pad_gap(theta) >= min(open_width, PAD_GAP_M[0]) - tolerance
