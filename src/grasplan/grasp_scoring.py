#!/usr/bin/env python3

# Copyright (c) 2026 DFKI GmbH
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

'''
Ranking of external (AnyGrasp-style) grasp candidates.

The AnyGrasp action server and its recorded-grasp mockup share this module so
both rank candidates identically: the raw network score ("nn") is multiplied
by a top-down bonus that grows with how well the approach direction points
straight down in a gravity-aligned frame. Every function is ROS-free apart
from the GraspCandidate message fields it reads and writes.
'''

import math

from grasplan.grasp_markers import make_candidate_scores

NN_SCORE_NAME = 'nn'


def top_down_alignment(approach_vector):
    '''Fraction in [0, 1] of how well an approach vector (x, y, z) points along -z of its frame.'''
    x, y, z = approach_vector
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0.0:
        return 0.0
    return max(0.0, min(1.0, -z / norm))


def top_down_multiplier(alignment, score_multiplier):
    '''Linear bonus: 1.0 for a horizontal approach, score_multiplier for a vertical one.'''
    return 1.0 + (score_multiplier - 1.0) * alignment


def nn_score(candidate):
    '''The raw network score of a candidate, falling back to quality for legacy producers.'''
    for score in getattr(candidate, 'scores', []):
        if score.name == NN_SCORE_NAME:
            return float(score.value)
    return float(candidate.quality)


def apply_top_down_bonus(candidates, approach_vectors, score_multiplier):
    '''
    Re-score candidates in place: quality = nn * top_down_multiplier, scores = [nn, top_bonus, final].

    approach_vectors: one (x, y, z) per candidate, expressed in the gravity-aligned frame.
    '''
    for candidate, approach in zip(candidates, approach_vectors):
        raw = nn_score(candidate)
        multiplier = top_down_multiplier(top_down_alignment(approach), score_multiplier)
        candidate.quality = raw * multiplier
        candidate.scores = make_candidate_scores(candidate.quality, nn=raw, top_bonus=multiplier)


def rank_by_quality(candidates):
    '''Return the candidates ordered best first; ties keep their input order.'''
    return sorted(candidates, key=lambda candidate: candidate.quality, reverse=True)
