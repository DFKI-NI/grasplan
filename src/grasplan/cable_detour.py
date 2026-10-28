'''
The one switch for the two exclusive ways to keep the arm cable out of trouble, read by the pick, place and insert nodes
(rosparam ~cable_detour_mode):

- hack (the code default): the closed-set objects go through the fixed untangle poses (untangle_cable_guide_1_right,
  then _2_right) before the pick, the place and the insert, as always. The MTC cable guard still checks every plan.
- cable_aware: no untangle pose at all, not before a pick, a place or an insert and not as a fallback after a refusal
  (so also no cable replay detour). The MTC cable guard and its planning help are the only protection; a motion the
  guard refuses every plan for fails, and the failure text says so.

Careful flag: the launch files and the GUI option switch cable_aware on in the sim; the real robot stays on hack until the
user's supervised test.
'''
import re

import rospy

HACK = 'hack'
CABLE_AWARE = 'cable_aware'
_warned = set()


def cable_detour_mode():
    '''hack or cable_aware; any other value is reported once and counts as hack (the untangle poses stay)'''
    mode = str(rospy.get_param('~cable_detour_mode', HACK)).strip().lower()
    if mode in (HACK, CABLE_AWARE):
        return mode
    if mode not in _warned:
        _warned.add(mode)
        rospy.logwarn(f'cable_detour_mode {mode!r} is neither {HACK} nor {CABLE_AWARE}: using {HACK}')
    return HACK


def requested_cable_aware():
    '''whether cable_aware is asked for (the parameter alone, without looking for a guard)'''
    return cable_detour_mode() == CABLE_AWARE


def cable_aware(mtc, what='motion'):
    '''
    whether the cable-aware mode is in force: asked for AND the MTC cable guard is active (mtc is the node's MtcPickPlace,
    ~mtc_cable_plan_check true and the model available). Cable-aware without the guard would leave neither the untangle
    poses nor the guard, so then the poses stay (hack) and a warning says why.
    '''
    if not requested_cable_aware():
        return False
    if mtc is not None:
        mtc.update_cable_guard()
    if mtc is None or getattr(mtc, 'cable_guard', None) is None:
        rospy.logwarn(f'cable_detour_mode is {CABLE_AWARE} but no MTC cable guard is active (use_mtc and '
                      f'mtc_cable_plan_check needed): the untangle poses stay for the {what}')
        return False
    return True


def explain_refusal(reason):
    '''
    the failure reason of a pick, place or insert; in cable_aware mode a reason that names the cable guard's refusal gets
    the sentence that this mode visits no untangle pose, so the person knows why the arm stayed where it was
    '''
    if 'refused by the cable guard' not in (reason or '') or not requested_cable_aware():   # reads the parameter only when needed
        return reason
    refused = sum(int(n) for n in re.findall(r'(\d+) refused by the cable guard', reason))
    count = f'{refused} plan(s)' if refused else 'the plans'
    return (f'{reason}; cable-aware mode: the cable guard refused {count} and this mode visits no untangle pose, '
            f'so the arm stayed where it was (cable_detour_mode hack would use the untangle poses first)')
