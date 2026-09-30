import math
import unittest

import numpy as np

from grasplan.tools.touching_boxes import boxes_touch


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    result = np.eye(4)
    result[:2, :2] = [[math.cos(yaw), -math.sin(yaw)],
                      [math.sin(yaw), math.cos(yaw)]]
    result[:3, 3] = [x, y, z]
    return result


class TouchingBoxesTest(unittest.TestCase):
    def test_margin_and_vertical_clearance(self):
        box = (0.2, 0.2, 0.1)
        self.assertTrue(boxes_touch(pose(), box, pose(x=0.2), box))
        self.assertTrue(boxes_touch(pose(), box, pose(x=0.209), box, 0.01))
        self.assertFalse(boxes_touch(pose(), box, pose(x=0.211), box, 0.01))
        self.assertFalse(boxes_touch(pose(), box, pose(z=0.12), box, 0.01))

    def test_rotated_boxes_use_shape_not_aabb(self):
        bar = (1.0, 0.1, 0.1)
        self.assertTrue(boxes_touch(pose(yaw=math.pi / 4), bar,
                                    pose(y=0.08, yaw=math.pi / 4), bar))
        self.assertFalse(boxes_touch(pose(yaw=math.pi / 4), bar,
                                     pose(x=-0.15, y=0.15, yaw=math.pi / 4), bar))


if __name__ == '__main__':
    unittest.main()
