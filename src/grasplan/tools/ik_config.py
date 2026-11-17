'''
Which IK solver settings the MTC planning of this node runs with, for one log line at node start. Log only.

MTC plans in the C++ side of the Python bindings (node ``<node>_cpp``), which loads the robot model and its IK solver
plugins once, at start, from the parameter server, the way MoveIt's kinematics plugin loader looks them up from that
node's private namespace: first ``<group>/kinematics_solver`` searched upwards from ``<node>_cpp`` (a private copy of a
kinematics.yaml there wins), else ``<found>_kinematics/<group>/kinematics_solver`` read at an absolute name, where
``<found>`` is ``robot_description`` searched upwards from ``<node>_cpp`` (the robot model the node loaded; the entry
move_group's planning context loads). The timeout and solve type are read next to the solver key that was found.
in_force() repeats that lookup; master_default() reads the robot_description_kinematics entry alone, so one line shows
both where they differ (e.g. a robot whose own MoveIt config holds another timeout than this checkout).
'''


def _found(search, has, ns, key):
    '''full name of key searched upwards from ns as roscpp's NodeHandle::searchParam does, None when absent'''
    try:
        name = search(ns, key)
    except Exception:  # noqa: BLE001 - the master answers 'not found' with an error
        return None
    return name if name and has(name) else None


def _entry(has, get, name):
    '''{solver, timeout, solve_type, param} of the solver key name; None when absent'''
    if name is None or not has(name):
        return None
    base = name[:-len('/kinematics_solver')]
    return {'solver': get(name, None), 'timeout': get(base + '/kinematics_solver_timeout', None),
            'solve_type': get(base + '/solve_type', None), 'param': base}


def private_entry(search, has, get, ns, group):
    '''the private-style entry: <group>/kinematics_solver searched upwards from ns'''
    return _entry(has, get, _found(search, has, ns, f'{group}/kinematics_solver'))


def master_default(search, has, get, ns, group, description='robot_description'):
    '''the <description>_kinematics entry of the robot model ns loads (description searched upwards from ns, the
    kinematics entry read at the absolute name beside it): the master's MoveIt config'''
    found = _found(search, has, ns, description)
    return None if found is None else _entry(has, get, f'{found}_kinematics/{group}/kinematics_solver')


def in_force(search, has, get, ns, group, description='robot_description'):
    '''the entry MoveIt's loader takes for group in node ns: the private-style one first, else the master's
    <description>_kinematics one; None when neither exists'''
    return (private_entry(search, has, get, ns, group)
            or master_default(search, has, get, ns, group, description))


def text(entry):
    if entry is None:
        return 'none set (MoveIt defaults)'
    timeout = entry['timeout']
    timeout = 'timeout unset' if timeout is None else f'timeout {timeout} s'
    solve = f", solve_type {entry['solve_type']}" if entry['solve_type'] is not None else ''
    return f"{entry['solver']}, {timeout}{solve} (from {entry['param']})"


def log_line(ns, group, search, has, get):
    '''one start line: the IK settings in force for group in node ns and the master's robot_description_kinematics'''
    used = in_force(search, has, get, ns, group)
    master = master_default(search, has, get, ns, group)
    same = used is not None and master is not None and used['param'] == master['param']
    return (f'mtc: IK of group {group} in {ns}: {text(used)}; master robot_description_kinematics: '
            + ('the same entry' if same else text(master)))


def ros_access():
    '''search / has / get on the live parameter server; search(ns, key) asks the master with ns as the caller, the way
    roscpp's NodeHandle::searchParam of a node handle in ns does'''
    import rosgraph
    import rospy
    return (lambda ns, key: rosgraph.Master(ns).searchParam(key)), rospy.has_param, rospy.get_param
