'''shape_features: generic features from synthetic point clouds; the dimensions below are test-side ground truth only.'''
import unittest

import numpy as np

from grasplan import shape_features as sf

RNG_SEED = 7


def noise(points, sigma=0.0015):
    return points + np.random.default_rng(RNG_SEED).normal(0.0, sigma, points.shape)


def cylinder_front(radius, height, n=3000, half=True, support=0.0, axis_xy=(0.0, 0.0)):
    '''points on the camera-facing half (or all) of a standing cylinder's side surface'''
    rng = np.random.default_rng(RNG_SEED)
    angle = rng.uniform(-np.pi / 2, np.pi / 2, n) if half else rng.uniform(-np.pi, np.pi, n)
    z = rng.uniform(support, support + height, n)
    return np.column_stack([axis_xy[0] + radius * np.cos(angle), axis_xy[1] + radius * np.sin(angle), z])


def sphere_front(radius, n=3000, support=0.0):
    '''the camera-facing hemisphere of a ball resting on the support'''
    rng = np.random.default_rng(RNG_SEED)
    direction = rng.normal(size=(n, 3))
    direction /= np.linalg.norm(direction, axis=1, keepdims=True)
    direction = direction[direction[:, 0] >= 0.0]
    return direction * radius + np.array([0.0, 0.0, support + radius])


def lying_cylinder(radius, length, n=4000, support=0.0):
    '''the camera-facing half of a cylinder lying along y (the camera looks along x)'''
    rng = np.random.default_rng(RNG_SEED)
    angle = rng.uniform(-np.pi / 2, np.pi / 2, n)
    y = rng.uniform(-length / 2, length / 2, n)
    return np.column_stack([radius * np.cos(angle), y, support + radius + radius * np.sin(angle)])


def box_two_faces(size, n=4000, support=0.0):
    '''front (x = 0) and left (y = 0) faces of a box seen from a corner, size = (x, y, z)'''
    rng = np.random.default_rng(RNG_SEED)
    sx, sy, sz = size
    front = np.column_stack([np.zeros(n // 2), rng.uniform(0, sy, n // 2), rng.uniform(support, support + sz, n // 2)])
    side = np.column_stack([rng.uniform(0, sx, n // 2), np.zeros(n // 2), rng.uniform(support, support + sz, n // 2)])
    return np.vstack([front, side])


class ShapeFeaturesTest(unittest.TestCase):
    def test_half_seen_standing_can_gives_its_full_diameter(self):
        f = sf.shape_features(noise(cylinder_front(0.0375, 0.24)), 0.0)
        self.assertEqual(f['shape'], 'cylinder')
        self.assertTrue(f['round'] and f['outline_reliable'] and f['height_reliable'])
        self.assertAlmostEqual(f['diameter'], 0.075, delta=0.0075)
        self.assertAlmostEqual(f['height'], 0.24, delta=0.01)
        self.assertTrue(f['tall'] and not f['flat'] and not f['elongated'] and f['graspable_across'])

    def test_a_ball_is_sphere_like_for_two_sizes(self):
        for diameter in (0.04, 0.07):
            f = sf.shape_features(noise(sphere_front(diameter / 2)), 0.0)
            self.assertEqual(f['shape'], 'sphere', diameter)
            self.assertAlmostEqual(f['diameter'], diameter, delta=0.1 * diameter)
            self.assertAlmostEqual(f['height'], diameter, delta=0.1 * diameter)
            self.assertFalse(f['tall'])

    def test_a_lying_can_is_a_lying_cylinder_graspable_across_its_short_axis(self):
        f = sf.shape_features(noise(lying_cylinder(0.0375, 0.2)), 0.0)
        self.assertEqual(f['shape'], 'lying_cylinder')
        self.assertAlmostEqual(f['footprint_min'], 0.075, delta=0.0075)
        self.assertAlmostEqual(f['footprint_max'], 0.2, delta=0.02)
        self.assertTrue(f['elongated'] and f['graspable_across'] and not f['tall'])

    def test_a_flat_box_is_flat_and_box_like(self):
        f = sf.shape_features(noise(box_two_faces((0.06, 0.10, 0.02)), 0.0005), 0.0)
        self.assertEqual(f['shape'], 'box')
        self.assertFalse(f['round'])
        self.assertTrue(f['flat'] and f['height_reliable'])
        self.assertAlmostEqual(f['height'], 0.02, delta=0.004)

    def test_a_long_bar_is_elongated(self):
        f = sf.shape_features(noise(box_two_faces((0.05, 0.25, 0.05)), 0.0005), 0.0)
        self.assertEqual(f['shape'], 'box')
        self.assertTrue(f['elongated'])
        self.assertAlmostEqual(f['footprint_max'], 0.25, delta=0.02)

    def test_a_sparse_blob_is_unreliable_not_guessed(self):
        f = sf.shape_features(noise(cylinder_front(0.03, 0.1, n=60)), 0.0)
        self.assertEqual(f['shape'], 'unknown')
        self.assertFalse(f['outline_reliable'] or f['height_reliable'])
        self.assertIsNone(f['diameter'])
        self.assertTrue(f['reasons'])

    def test_a_box_seen_face_on_leaves_the_other_side_unknown(self):
        face = np.column_stack([np.zeros(2000), np.random.default_rng(1).uniform(0, 0.1, 2000),
                                np.random.default_rng(2).uniform(0, 0.08, 2000)])
        f = sf.shape_features(noise(face, 0.0003), 0.0)
        self.assertEqual(f['shape'], 'box')
        self.assertIsNone(f['footprint_min'])
        self.assertIsNone(f['graspable_across'])

    def test_completed_depth_keeps_the_outline_but_not_the_height(self):
        f = sf.shape_features(noise(cylinder_front(0.0375, 0.24)), 0.0, completed_depth=True)
        self.assertTrue(f['outline_reliable'])
        self.assertFalse(f['height_reliable'])
        self.assertIsNone(f['tall'])
        self.assertIsNone(f['flat'])

    def test_the_height_counts_from_the_support_plane_not_the_cloud_bottom(self):
        # a single view's bottom edge lies above the table: the cloud starts 3 cm up, the support is at 0
        cloud = cylinder_front(0.0375, 0.21, support=0.03)
        f = sf.shape_features(noise(cloud), 0.0)
        self.assertAlmostEqual(f['height'], 0.24, delta=0.01)

    def test_too_wide_for_the_gripper_is_not_graspable_across(self):
        f = sf.shape_features(noise(cylinder_front(0.08, 0.2)), 0.0)
        self.assertEqual(f['shape'], 'cylinder')
        self.assertFalse(f['graspable_across'])

    def test_the_feature_dict_names_no_object(self):
        f = sf.shape_features(noise(cylinder_front(0.0375, 0.24)), 0.0)
        self.assertFalse([k for k in f if 'name' in k or 'class' in k])

    def test_the_classes_hold_for_other_noise_draws(self):
        global RNG_SEED
        try:
            for seed in range(1, 11):
                RNG_SEED = seed
                self.assertEqual(sf.shape_features(noise(cylinder_front(0.0375, 0.24), 0.001), 0.0)['shape'], 'cylinder', seed)
                self.assertEqual(sf.shape_features(noise(sphere_front(0.035), 0.001), 0.0)['shape'], 'sphere', seed)
                self.assertEqual(sf.shape_features(noise(lying_cylinder(0.0375, 0.2), 0.001), 0.0)['shape'],
                                 'lying_cylinder', seed)
                self.assertEqual(sf.shape_features(noise(box_two_faces((0.05, 0.25, 0.05)), 0.0005), 0.0)['shape'], 'box', seed)
        finally:
            RNG_SEED = 7


if __name__ == '__main__':
    unittest.main()
