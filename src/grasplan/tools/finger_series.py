'''
The closing of the gripper as a measurement: from the finger joint's samples (time, angle rad, effort or None)
between the close command and the settle, the angle where the fingers first stopped (contact), where they ended and the
creep between the two (fingers still closing on a soft or sliding object), plus the effort at the end. No object knowledge:
only what the gripper itself reports. Pure functions, no ROS.
'''
import math

STILL_SPEED = 0.05   # rad/s: slower than this the fingers count as stopped
MIN_TRAVEL = 0.02    # rad: the fingers must have moved this far from the open angle before a stop counts as contact


def analyse_close(samples, still_speed=STILL_SPEED, min_travel=MIN_TRAVEL):
    '''
    dict with contact (rad: first stop after min_travel, else the last angle), final (rad: mean of the last 3 samples),
    creep (rad, final - contact, 0 or more), contact_time (s after the first sample), effort (last effort or None),
    peak_effort (largest |effort| or None), n (samples), duration (s); None when fewer than 3 samples
    '''
    if not samples or len(samples) < 3:
        return None
    start = samples[0]
    contact_index = len(samples) - 1
    for i in range(1, len(samples)):
        dt = samples[i][0] - samples[i - 1][0]
        if dt <= 0.0:
            continue
        speed = abs(samples[i][1] - samples[i - 1][1]) / dt
        if speed < still_speed and abs(samples[i][1] - start[1]) >= min_travel:
            contact_index = i
            break
    final = sum(s[1] for s in samples[-3:]) / 3.0
    contact = samples[contact_index][1]
    efforts = [s[2] for s in samples if s[2] is not None and not math.isnan(s[2])]
    return {'contact': contact, 'final': final, 'creep': max(final - contact, 0.0),
            'contact_time': samples[contact_index][0] - start[0], 'effort': efforts[-1] if efforts else None,
            'peak_effort': max(abs(e) for e in efforts) if efforts else None, 'n': len(samples),
            'duration': samples[-1][0] - start[0]}


def closing_axis_tilt_deg(rotation):
    '''angle (deg) of the pad closing axis (TCP y) out of the horizontal plane, from the 3x3 TCP rotation in a z-up frame.
    A grasp tilted by this angle reads a wider gap than the object across its pads, by design.'''
    return math.degrees(math.asin(max(-1.0, min(1.0, abs(rotation[2][1])))))
