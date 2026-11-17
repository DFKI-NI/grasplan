'''start log line of the MTC nodes: the IK settings the C++ side loads (MoveIt's kinematics plugin loader lookup from
<node>_cpp: a private <group>/kinematics_solver first, else robot_description_kinematics/<group>) and the master's
robot_description_kinematics entry. Fake parameter server with the rosmaster search rule; no ROS master needed.'''
import unittest

from grasplan.tools import ik_config

NODE = '/mobipick/pick_object_node_cpp'
TRAC = 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin'


class Params:
    '''flat parameter dict; has() also knows namespaces; search() as rosmaster's search_param'''

    def __init__(self, values):
        self.values = values

    def has(self, name):
        return name in self.values or any(k.startswith(name.rstrip('/') + '/') for k in self.values)

    def get(self, name, default=None):
        return self.values.get(name, default)

    def search(self, ns, key):
        parts = [x for x in key.split('/') if x]
        spaces = [x for x in ns.split('/') if x]
        for i in range(0, len(spaces) + 1):
            base = spaces[:len(spaces) - i]
            if self.has('/' + '/'.join(base + parts[:1])):
                return '/' + '/'.join(base + parts)
        raise RuntimeError('not found')   # the master answers with an error

    def line(self, group='arm'):
        return ik_config.log_line(NODE, group, self.search, self.has, self.get)


def entry(base, timeout, solver=TRAC, solve_type='Distance'):
    out = {f'{base}/kinematics_solver': solver, f'{base}/solve_type': solve_type}
    if '_kinematics/' in base:   # the robot model beside a master entry
        out[base.split('_kinematics/')[0]] = '<robot/>'
    if timeout is not None:
        out[f'{base}/kinematics_solver_timeout'] = timeout
    return out


MASTER = '/mobipick/robot_description_kinematics/arm'


class TestIkConfigLine(unittest.TestCase):
    def test_sim_today_master_entry_in_force(self):
        p = Params(entry(MASTER, 0.01))
        used = ik_config.in_force(p.search, p.has, p.get, NODE, 'arm')
        self.assertEqual(used['timeout'], 0.01)
        self.assertEqual(used['param'], MASTER)
        self.assertEqual(p.line(), f'mtc: IK of group arm in {NODE}: {TRAC}, timeout 0.01 s, solve_type Distance '
                                   f'(from {MASTER}); master robot_description_kinematics: the same entry')

    def test_private_copy_wins_over_the_robot_masters_value(self):
        values = entry(MASTER, 0.1)
        values.update(entry(NODE + '/arm', 0.01))
        p = Params(values)
        used = ik_config.in_force(p.search, p.has, p.get, NODE, 'arm')
        self.assertEqual((used['timeout'], used['param']), (0.01, NODE + '/arm'))
        line = p.line()
        self.assertIn(f'timeout 0.01 s, solve_type Distance (from {NODE}/arm)', line)
        self.assertIn(f'master robot_description_kinematics: {TRAC}, timeout 0.1 s, solve_type Distance (from {MASTER})',
                      line)

    def test_a_private_timeout_alone_is_not_read(self):
        values = entry(MASTER, 0.1)
        values[NODE + '/arm/kinematics_solver_timeout'] = 0.01
        p = Params(values)
        self.assertEqual(ik_config.in_force(p.search, p.has, p.get, NODE, 'arm')['timeout'], 0.1)
        self.assertIn('the same entry', p.line())

    def test_a_group_namespace_higher_up_without_solver_falls_back(self):
        values = entry(MASTER, 0.1)
        values['/mobipick/arm/something_else'] = 1
        p = Params(values)
        self.assertEqual(ik_config.in_force(p.search, p.has, p.get, NODE, 'arm')['param'], MASTER)

    def test_nothing_set(self):
        p = Params({})
        self.assertEqual(p.line(), f'mtc: IK of group arm in {NODE}: none set (MoveIt defaults); '
                                   'master robot_description_kinematics: none set (MoveIt defaults)')

    def test_unset_timeout_is_named(self):
        p = Params(entry(MASTER, None))
        self.assertIn('timeout unset', p.line())

    def test_the_master_entry_sits_beside_the_robot_description_found(self):
        values = entry(MASTER, 0.1)
        values['/robot_description_kinematics/arm/kinematics_solver'] = 'kdl'   # a global one is not the robot's
        values['/robot_description_kinematics/arm/kinematics_solver_timeout'] = 0.5
        p = Params(values)
        self.assertEqual(ik_config.in_force(p.search, p.has, p.get, NODE, 'arm')['param'], MASTER)

    def test_a_kinematics_entry_without_its_robot_description_is_not_read(self):
        values = {k: v for k, v in entry(MASTER, 0.1).items() if k != '/mobipick/robot_description'}
        values['/robot_description'] = '<robot/>'   # the robot model found is the global one: no entry beside it
        p = Params(values)
        self.assertIsNone(ik_config.in_force(p.search, p.has, p.get, NODE, 'arm'))

    def test_other_group_name(self):
        p = Params(entry('/mobipick/robot_description_kinematics/manipulator', 0.01))
        self.assertIn('timeout 0.01 s', p.line('manipulator'))
        self.assertIn('none set', p.line('arm'))


if __name__ == '__main__':
    unittest.main()
