'''grasp_scoring: side-grasp bonus for long cylinders (standing Pringles can, 2026-09-28).'''
import math
from types import SimpleNamespace as NS

import pytest

from grasplan import grasp_scoring as gs

PRINGLES = (0.086, 0.099, 0.237)


def test_long_cylinders_by_shape_not_height():
    axis, length = gs.cylinder_axis(PRINGLES)
    assert axis == (0.0, 0.0, 1.0) and length == pytest.approx(0.237)
    assert gs.cylinder_axis((0.066, 0.066, 0.12))[0] == (0.0, 0.0, 1.0)      # coke can
    assert gs.cylinder_axis((0.237, 0.086, 0.099))[0] == (1.0, 0.0, 0.0)     # the can lying along x
    assert gs.cylinder_axis((0.04, 0.09, 0.17)) is None                      # sugar box: flat cross-section
    assert gs.cylinder_axis((0.07, 0.07, 0.07)) is None                      # tennis ball
    assert gs.cylinder_axis((0.3, 0.2, 0.15)) is None                        # klt
    assert gs.cylinder_axis((0.0, 0.1, 0.3)) is None
    turned = ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))            # box z axis along world x
    assert gs.cylinder_axis(PRINGLES, turned)[0] == (1.0, 0.0, 0.0)


def test_side_multiplier_prefers_across_and_middle():
    up, length = (0.0, 0.0, 1.0), 0.237
    side = gs.cylinder_side_multiplier((1, 0, 0), (0, 1, 0), (0, 0, 0.02), up, length)
    assert side == pytest.approx(gs.SIDE_GRASP_SCORE_MULTIPLIER)
    assert gs.cylinder_side_multiplier((0, 0, -1), (0, 1, 0), (0, 0, 0.1), up, length) == pytest.approx(1.0)  # top
    assert gs.cylinder_side_multiplier((1, 0, 0), (0, 0, 1), (0, 0, 0), up, length) == pytest.approx(1.0)  # fingers along axis
    near_end = gs.cylinder_side_multiplier((1, 0, 0), (0, 1, 0), (0, 0, 0.1), up, length)
    assert 1.0 < near_end < side
    tilted = gs.cylinder_side_multiplier((math.cos(0.5), 0, -math.sin(0.5)), (0, 1, 0), (0, 0, 0), up, length)
    assert 1.0 < tilted < side
    # a lying can: top-down across it is a side grasp of the cylinder
    assert gs.cylinder_side_multiplier((0, 0, -1), (0, 1, 0), (0, 0, 0), (1, 0, 0), length) == pytest.approx(1.5)


def test_apply_cylinder_side_bonus_reranks():
    saved, gs.make_candidate_scores = gs.make_candidate_scores, lambda final, **named: dict(named, final=final)
    top = NS(quality=0.8, scores=[NS(name='nn', value=0.8)])
    side = NS(quality=0.6, scores=[NS(name='nn', value=0.6)])
    gs.apply_cylinder_side_bonus([top, side], [((0, 0, -1), (0, 1, 0), (1, 1, 0.9)), ((1, 0, 0), (0, 1, 0), (1, 1, 0.85))],
                                 (1.0, 1.0, 0.84), (0.0, 0.0, 1.0), 0.237)
    assert top.quality == pytest.approx(0.8) and side.quality == pytest.approx(0.9)
    assert side.scores['side_bonus'] == pytest.approx(1.5)
    gs.make_candidate_scores = saved


def test_side_or_top_by_size_and_shape():
    assert gs.side_grasp_axis(PRINGLES)[0] == (0.0, 0.0, 1.0)                  # long cylinder: its axis
    axis, height = gs.side_grasp_axis((0.074, 0.075, 0.073))                   # baseball: side at centre height
    assert axis == (0.0, 0.0, 1.0) and height == pytest.approx(0.073)
    assert gs.side_grasp_axis((0.067, 0.066, 0.068)) is not None              # tennis ball
    assert gs.side_grasp_axis((0.05, 0.05, 0.05)) is None                      # tiny: strict top-down
    assert gs.side_grasp_axis((0.056, 0.078, 0.062)) is None                   # strawberry box: not round enough
    assert gs.side_grasp_axis((0.04, 0.2, 0.05)) is None                       # banana: a lying cylinder
    assert gs.side_grasp_axis((0.237, 0.086, 0.099)) is None                   # Pringles on its side: top-down
    assert gs.side_grasp_axis((0.08, 0.17, 0.04)) is None                      # multimeter: flat
    assert gs.side_grasp_axis((0.3, 0.2, 0.15)) is None                        # klt
