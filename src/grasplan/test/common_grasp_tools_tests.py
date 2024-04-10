#!/usr/bin/env python3

# Copyright (c) 2024 DFKI GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import unittest
from grasplan.common_grasp_tools import separate_object_class_from_id


class TestCommonGraspTools(unittest.TestCase):
    def test_normal(self):
        self.assertEquals(separate_object_class_from_id('relay_1'), ('relay', 1))

    def test_no_id(self):
        self.assertEquals(separate_object_class_from_id('relay'), ('relay', None))

    def test_underscore(self):
        self.assertEquals(separate_object_class_from_id('power_drill_with_grip'), ('power_drill_with_grip', None))

    def test_underscore_with_id(self):
        self.assertEquals(separate_object_class_from_id('power_drill_with_grip_3'), ('power_drill_with_grip', 3))

    def test_underscore_with_id_and_num_middle(self):
        self.assertEquals(separate_object_class_from_id('power_drill2_34with_grip_3'), ('power_drill2_34with_grip', 3))

    def test_klt_2(self):
        self.assertEquals(separate_object_class_from_id('klt_2'), ('klt', 2))

    def test_klt_1(self):
        self.assertEquals(separate_object_class_from_id('klt_1'), ('klt', 1))

    def test_power_drill_with_grip_1(self):
        self.assertEquals(separate_object_class_from_id('power_drill_with_grip_1'), ('power_drill_with_grip', 1))

    def test_power_drill(self):
        self.assertEquals(separate_object_class_from_id('power_drill'), ('power_drill', None))

    def test_screwdriver_1(self):
        self.assertEquals(separate_object_class_from_id('screwdriver_1'), ('screwdriver', 1))

    def test_screwdriver(self):
        self.assertEquals(separate_object_class_from_id('screwdriver'), ('screwdriver', None))

    def test_relay_1(self):
        self.assertEquals(separate_object_class_from_id('relay_1'), ('relay', 1))

    def test_klt_3(self):
        self.assertEquals(separate_object_class_from_id('klt_3'), ('klt', 3))

    def test_multimeter_1(self):
        self.assertEquals(separate_object_class_from_id('multimeter_1'), ('multimeter', 1))

    def test_multimeter(self):
        self.assertEquals(separate_object_class_from_id('multimeter'), ('multimeter', None))

    def test_only_class_required(self):
        self.assertEquals(separate_object_class_from_id('multimeter')[0], 'multimeter')


if __name__ == '__main__':
    import rostest

    rostest.rosrun('grasplan', 'test_cgt', TestCommonGraspTools)
