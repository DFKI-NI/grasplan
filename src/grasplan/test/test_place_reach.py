'''unit tests of grasplan.tools.place_reach (#132, no ROS): python3 -m pytest <this file>'''
from types import SimpleNamespace

from grasplan.tools.place_reach import order_by_reach


def obj(x, y):
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.7)))


def test_nearest_first():
    objects = [obj(1.0, 0.0), obj(0.3, 0.1), obj(0.0, -0.6)]
    ordered, dropped = order_by_reach(objects, (0.0, 0.0))
    assert [o.pose.position.x for o in ordered] == [0.3, 0.0, 1.0] and dropped == 0


def test_base_offset_and_cutoff():
    objects = [obj(2.0, 1.0), obj(1.5, 1.0), obj(2.9, 1.0)]
    ordered, dropped = order_by_reach(objects, (1.0, 1.0), max_reach=1.2)
    assert [o.pose.position.x for o in ordered] == [1.5, 2.0] and dropped == 1


def test_cutoff_keeps_all_when_none_is_within_reach():
    objects = [obj(2.0, 0.0), obj(1.5, 0.0)]
    ordered, dropped = order_by_reach(objects, (0.0, 0.0), max_reach=0.5)
    assert [o.pose.position.x for o in ordered] == [1.5, 2.0] and dropped == 0
