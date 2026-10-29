'''
Arm paths the cable model shapes before planning: constraints engineered and decided by the cable model, not plans
made freely and then rejected.

The geometric stretch of a cable span depends only on the joints between its clamps (the wrist span: wrist_1, wrist_2,
wrist_3; the big joints only turn the carrier chain's gravity direction, which the guard's chain replay judges). So the
stretch of each span is tabulated once on a joint grid (CableGrid, cached in a file keyed by a hash of the model
config) and everything at planning time is a lookup:

- slack_path(): the shortest joint path from start to goal whose stretch stays below a margin (Dijkstra on the slack
  grid cells, then shortcut), as waypoints; the straight joint line when it is slack already. The joints no span
  depends on move linearly along it.
- corridor(): per path segment a box on the span joints around the segment, every grid node in it slack (a search bias
  for a free planner, never a proof: the guard still checks the planned path).
- ur_branch_seeds() / rank_branches(): seeds for every UR IK branch of a pose and their order by predicted stretch, so
  planning starts from the branch with the most slack.

Pure python (numpy, scipy optional); the model (mobipick_cable_entanglement.CableModel) is needed only to build a
grid. Joint names are keyed short ('wrist_1'), as the model accepts them with or without prefix, 'ur5_' and '_joint'.
'''
import hashlib
import heapq
import json
import math
import os

import numpy as np

GRID_STEP_DEG = 10.0
GRID_VERSION = 1


def short_name(joint):
    '''mobipick/ur5_wrist_1_joint -> wrist_1'''
    return joint.split('/')[-1].replace('ur5_', '').replace('_joint', '')


def grid_key(config_dict, urdf, step_deg, limits, extra=''):
    '''hash of everything a grid depends on: model config, robot description, step, joint limits'''
    h = hashlib.sha1()
    h.update(json.dumps(config_dict, sort_keys=True, default=str).encode())
    h.update(urdf.encode())
    h.update(json.dumps({'step': step_deg, 'limits': sorted((k, round(a, 6), round(b, 6))
                                                            for k, (a, b) in limits.items()),
                         'version': GRID_VERSION, 'extra': extra}).encode())
    return h.hexdigest()[:16]


def default_cache_path(key):
    '''$ROS_HOME/grasplan/cable_grid_<key>.npz (ROS_HOME defaults to ~/.ros)'''
    home = os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros')
    return os.path.join(home, 'grasplan', f'cable_grid_{key}.npz')


class SpanGrid:
    '''the stretch of one span on a regular grid over its joints (radians)'''

    def __init__(self, name, joints, axes, values):
        self.name = name
        self.joints = list(joints)
        self.axes = [np.asarray(a, dtype=float) for a in axes]
        self.values = np.asarray(values, dtype=float)
        self.lo = np.array([a[0] for a in self.axes])
        self.hi = np.array([a[-1] for a in self.axes])

    def point(self, q):
        return np.array([q[j] for j in self.joints], dtype=float)

    def _cell(self, x):
        '''lower node index per axis of the cell holding x (clamped), and the fraction inside it'''
        idx, frac = [], []
        for a, v in zip(self.axes, x):
            v = min(max(v, a[0]), a[-1])
            i = int(np.searchsorted(a, v, side='right')) - 1
            i = min(max(i, 0), len(a) - 2) if len(a) > 1 else 0
            f = 0.0 if len(a) == 1 else (v - a[i]) / (a[i + 1] - a[i])
            idx.append(i)
            frac.append(f)
        return idx, frac

    def _corners(self, idx):
        d = len(idx)
        for corner in range(1 << d):
            yield tuple(min(idx[k] + ((corner >> k) & 1), len(self.axes[k]) - 1) for k in range(d))

    def stretch(self, x):
        '''multilinear interpolation of the stretch at x (span joints, radians)'''
        idx, frac = self._cell(x)
        total = 0.0
        for node in self._corners(idx):
            w = 1.0
            for k, (i, f) in enumerate(zip(idx, frac)):
                w *= f if node[k] > i else (1.0 - f)
            total += w * self.values[node]
        return float(total)

    def stretch_upper(self, x):
        '''the largest node value of the cell holding x (conservative)'''
        idx, _ = self._cell(x)
        return float(max(self.values[node] for node in self._corners(idx)))

    def inside(self, x, tol=1e-9):
        return bool(np.all(x >= self.lo - tol) and np.all(x <= self.hi + tol))


class CableGrid:
    '''the stretch tables of all enabled spans; stretch(q) is the worst span, like CableState.max_stretch'''

    def __init__(self, spans, key=''):
        self.spans = list(spans)
        self.key = key
        self._graphs = {}

    @property
    def joints(self):
        return [j for s in self.spans for j in s.joints]

    def stretch(self, q):
        '''q: dict short joint name -> rad (other joints ignored); -inf without spans'''
        return max((s.stretch(s.point(q)) for s in self.spans), default=-float('inf'))

    def worst_along(self, a, b, step=math.radians(2.5)):
        '''max interpolated stretch on the joint line a -> b (dicts), sampled every step on the span joints'''
        return max((_line_worst(s, s.point(a), s.point(b), step) for s in self.spans), default=-float('inf'))

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        data = {'key': np.array(self.key), 'names': np.array([s.name for s in self.spans])}
        for i, s in enumerate(self.spans):
            data[f'joints_{i}'] = np.array(s.joints)
            data[f'values_{i}'] = s.values
            for k, a in enumerate(s.axes):
                data[f'axis_{i}_{k}'] = a
        tmp = path + '.tmp.npz'
        np.savez_compressed(tmp, **data)
        os.replace(tmp, path)   # readers never see a half-written grid

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as d:
            spans = []
            for i, name in enumerate(d['names']):
                joints = [str(j) for j in d[f'joints_{i}']]
                axes = [d[f'axis_{i}_{k}'] for k in range(len(joints))]
                spans.append(SpanGrid(str(name), joints, axes, d[f'values_{i}']))
            return cls(spans, str(d['key']))


def grid_axis(lo, hi, step):
    n = max(1, int(math.floor((hi - lo) / step + 1e-9)))
    axis = [lo + k * step for k in range(n + 1)]
    if hi - axis[-1] > 1e-6:
        axis.append(hi)
    return np.array(axis)


def build_grid(evaluate, span_joints, limits, step_deg=GRID_STEP_DEG, reference=None, key='', progress=None):
    '''
    evaluate(q) -> {span name: stretch} for a joint dict q (short names); span_joints: {span: [short joint names]};
    limits: {short joint name: (lo, hi)} radians; reference: values of the joints a span does not depend on.
    '''
    step = math.radians(step_deg)
    reference = dict(reference or {})
    spans = []
    for name, joints in span_joints.items():
        axes = [grid_axis(*limits[j], step) for j in joints]
        values = np.empty([len(a) for a in axes])
        total, done = values.size, 0
        for index in np.ndindex(values.shape):
            q = dict(reference)
            q.update({j: float(axes[k][i]) for k, (j, i) in enumerate(zip(joints, index))})
            values[index] = evaluate(q)[name]
            done += 1
            if progress is not None and done % 2000 == 0:
                progress(name, done, total)
        spans.append(SpanGrid(name, joints, axes, values))
    return CableGrid(spans, key)


def model_evaluator(model):
    '''CableModel -> evaluate(q) for build_grid: the stateless geometric stretch per span, without gradients'''
    def evaluate(q):
        state = model.update(q, 0.0, use_carrier=False, hysteresis=False, with_gradient=False, keep_state=False)
        return {n: s.stretch for n, s in state.spans.items()}
    return evaluate


def model_span_joints(model):
    '''{span name: short names of its wrap joints} of the spans the model computes'''
    return {s.name: [short_name(wj.joint) for wj in s.joints] for s in model.cfg.spans}


# ----------------------------------------------------------------------------------------------------------- search

def _stretch_many(span, X):
    '''multilinear stretch at the rows of X (M x d span-joint points), vectorized'''
    X = np.atleast_2d(np.asarray(X, dtype=float))
    d = X.shape[1]
    lower, frac = [], []
    for k, a in enumerate(span.axes):
        v = np.clip(X[:, k], a[0], a[-1])
        if len(a) == 1:
            lower.append(np.zeros(len(v), dtype=int))
            frac.append(np.zeros(len(v)))
            continue
        i = np.clip(np.searchsorted(a, v, side='right') - 1, 0, len(a) - 2)
        lower.append(i)
        frac.append((v - a[i]) / (a[i + 1] - a[i]))
    total = np.zeros(len(X))
    for corner in range(1 << d):
        w = np.ones(len(X))
        index = []
        for k in range(d):
            up = (corner >> k) & 1
            w *= frac[k] if up else (1.0 - frac[k])
            index.append(np.minimum(lower[k] + up, len(span.axes[k]) - 1))
        total += w * span.values[tuple(index)]
    return total


def _line_worst(span, p, q, step):
    n = max(1, int(math.ceil(float(np.max(np.abs(q - p))) / step))) if len(p) else 1
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    return float(_stretch_many(span, p + (q - p) * t).max())


def _graph(span, margin):
    '''(coords N x d, slack mask N, sparse graph or neighbour lists) of the span grid for this margin, cached'''
    cache = span.__dict__.setdefault('_graphs', {})
    key = round(float(margin), 6)
    if key in cache:
        return cache[key]
    shape = span.values.shape
    d = len(shape)
    coords = np.stack(np.meshgrid(*span.axes, indexing='ij'), axis=-1).reshape(-1, d)
    ok = (span.values <= margin).ravel()
    idx = np.indices(shape).reshape(d, -1).T
    src_all, dst_all, w_all = [], [], []
    for o in _neighbour_offsets(d):
        nb = idx + np.array(o)
        inside = np.all((nb >= 0) & (nb < np.array(shape)), axis=1)
        src = np.where(inside & ok)[0]
        dst = np.ravel_multi_index(nb[src].T, shape)
        keep = ok[dst]
        src, dst = src[keep], dst[keep]
        src_all.append(src)
        dst_all.append(dst)
        w_all.append(np.linalg.norm(coords[dst] - coords[src], axis=1))
    src, dst, w = np.concatenate(src_all), np.concatenate(dst_all), np.concatenate(w_all)
    try:
        from scipy.sparse import csr_matrix
        graph = csr_matrix((w, (src, dst)), shape=(len(coords), len(coords)))
    except ImportError:  # pure python fallback below
        graph = None
    cache[key] = (coords, ok, graph, (src, dst, w))
    return cache[key]


def _neighbour_offsets(d):
    offsets = []
    for o in np.ndindex(*([3] * d)):
        o = tuple(v - 1 for v in o)
        if any(o):
            offsets.append(o)
    return offsets


def _shortest(graph, edges, n, sources, source_cost):
    '''(dist, predecessor) over n nodes from several sources with initial costs'''
    try:
        from scipy.sparse.csgraph import dijkstra
        from scipy.sparse import csr_matrix, vstack, hstack
        # a virtual root joined to every source by its initial cost
        root_row = csr_matrix((source_cost, (np.zeros(len(sources), dtype=int), sources)), shape=(1, n + 1))
        g = vstack([hstack([graph, csr_matrix((n, 1))]), root_row]).tocsr()
        dist, pred = dijkstra(g, directed=True, indices=n, return_predecessors=True)
        return dist[:n], np.where(pred[:n] == n, -9999, pred[:n])
    except ImportError:
        src, dst, w = edges
        adjacency = {}
        for a, b, c in zip(src.tolist(), dst.tolist(), w.tolist()):
            adjacency.setdefault(a, []).append((b, c))
        dist = np.full(n, np.inf)
        pred = np.full(n, -9999)
        heap = []
        for s, c in zip(sources, source_cost):
            if c < dist[s]:
                dist[s] = c
                heapq.heappush(heap, (c, s))
        while heap:
            c, a = heapq.heappop(heap)
            if c > dist[a]:
                continue
            for b, w_ab in adjacency.get(a, ()):
                if c + w_ab < dist[b]:
                    dist[b], pred[b] = c + w_ab, a
                    heapq.heappush(heap, (c + w_ab, b))
        return dist, pred


def _span_path(span, xa, xb, margin, step):
    '''
    span-joint polyline xa -> xb (arrays) whose interpolated stretch stays <= the limit, or None; the limit is the margin,
    raised to an end's own stretch near that end (an arm already beyond the margin may leave, not go deeper)
    '''
    sa, sb = float(_stretch_many(span, xa)[0]), float(_stretch_many(span, xb)[0])
    if _line_worst(span, xa, xb, step) <= max(margin, sa, sb) + 1e-9:
        return [xa, xb]
    coords, ok, graph, edges = _graph(span, margin)
    free = np.where(ok)[0]
    if not len(free):
        return None

    def reachable(x, limit, count=8):
        order = free[np.argsort(np.linalg.norm(coords[free] - x, axis=1))[: count * 4]]
        out = [i for i in order if _line_worst(span, x, coords[i], step) <= limit + 1e-9]
        return out[:count]

    starts = reachable(xa, max(margin, sa))
    goals = reachable(xb, max(margin, sb))
    if not starts or not goals:
        return None
    dist, pred = _shortest(graph, edges, len(coords), np.array(starts),
                           np.linalg.norm(coords[starts] - xa, axis=1))
    goals = np.array(goals)
    total = dist[goals] + np.linalg.norm(coords[goals] - xb, axis=1)
    if not np.isfinite(total.min()):
        return None
    node, nodes = int(goals[int(np.argmin(total))]), []
    while node >= 0:
        nodes.append(coords[node])
        node = int(pred[node])
    points = [xa] + nodes[::-1] + [xb]

    def limit_between(i, j):
        lim = margin
        if i == 0:
            lim = max(lim, sa)
        if j == len(points) - 1:
            lim = max(lim, sb)
        return lim

    # shortcut: from each kept point jump to the farthest later point on a slack line
    kept, i = [points[0]], 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and _line_worst(span, points[i], points[j], step) > limit_between(i, j) + 1e-9:
            j -= 1
        kept.append(points[j])
        i = j
    return kept


def _fractions(polyline):
    lengths = [float(np.linalg.norm(b - a)) for a, b in zip(polyline, polyline[1:])]
    total = sum(lengths)
    if total <= 0.0:
        return [0.0, 1.0][: len(polyline)] if len(polyline) > 1 else [0.0]
    acc, out = 0.0, [0.0]
    for length in lengths:
        acc += length
        out.append(acc / total)
    return out


def _at(polyline, fractions, s):
    for k in range(1, len(polyline)):
        if s <= fractions[k] + 1e-12:
            span = fractions[k] - fractions[k - 1]
            t = 0.0 if span <= 0 else (s - fractions[k - 1]) / span
            return polyline[k - 1] + (polyline[k] - polyline[k - 1]) * t
    return polyline[-1]


def slack_path(grid, start, goal, margin, step=math.radians(2.5)):
    '''
    (waypoints, worst) from start to goal (joint dicts, short names, radians): waypoints are the intermediate joint
    states (dicts with every joint of goal; empty when the straight line is slack), worst the largest interpolated
    stretch along the whole path. None when the grid has no slack path. The joints no span depends on move linearly,
    in step with the span joints' progress.
    '''
    polylines = {}
    for span in grid.spans:
        xa, xb = span.point(start), span.point(goal)
        line = _span_path(span, xa, xb, margin, step)
        if line is None:
            return None
        polylines[span.name] = line
    breaks = sorted({round(f, 9) for line in polylines.values() for f in _fractions(line)})
    inner = [s for s in breaks if 1e-9 < s < 1 - 1e-9]
    waypoints = []
    for s in inner:
        q = {j: start[j] + (goal[j] - start[j]) * s for j in goal if j in start}
        q.update({j: goal[j] for j in goal if j not in start})
        for span in grid.spans:
            line = polylines[span.name]
            x = _at(line, _fractions(line), s)
            q.update({j: float(v) for j, v in zip(span.joints, x)})
        waypoints.append(q)
    chain = [start] + waypoints + [goal]
    worst = max(grid.worst_along(a, b, step) for a, b in zip(chain, chain[1:]))
    return waypoints, worst


def corridor(grid, a, b, margin, pad_deg=15.0, window=None):
    '''
    {short joint: (lo, hi)} on the span joints around the segment a -> b, padded by pad_deg and shrunk (15, 10, 5,
    0 deg) until every grid node in the box is slack (<= margin); window {joint: (lo, hi)} clips it (e.g. the wrist_3
    cable window). None when even the segment's own box holds a stretched node: then no corridor.
    '''
    window = window or {}
    for pad in (pad_deg, pad_deg * 2 / 3, pad_deg / 3, 0.0):
        p = math.radians(pad)
        box, ok = {}, True
        for span in grid.spans:
            lo_x = np.minimum(span.point(a), span.point(b)) - p
            hi_x = np.maximum(span.point(a), span.point(b)) + p
            sel = []
            for k, j in enumerate(span.joints):
                lo, hi = max(lo_x[k], span.lo[k]), min(hi_x[k], span.hi[k])
                if j in window:
                    lo, hi = max(lo, window[j][0]), min(hi, window[j][1])
                if lo > hi:
                    ok = False
                    break
                box[j] = (float(lo), float(hi))
                axis = span.axes[k]
                sel.append(np.where((axis >= lo - 1e-9) & (axis <= hi + 1e-9))[0])
            if not ok:
                break
            if all(len(s) for s in sel) and span.values[np.ix_(*sel)].max() > margin:
                ok = False
                break
        if ok:
            return box
    return None


# ----------------------------------------------------------------------------------------------------------- branches

UR_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']


def _wrap_into(v, lo, hi):
    '''v moved by multiples of 2 pi into [lo, hi] when possible (closest to the middle), else clamped'''
    mid = 0.5 * (lo + hi)
    best = None
    for k in range(-2, 3):
        w = v + 2 * math.pi * k
        if lo - 1e-9 <= w <= hi + 1e-9 and (best is None or abs(w - mid) < abs(best - mid)):
            best = w
    return min(max(v, lo), hi) if best is None else best


def ur_branch_seeds(q, limits):
    '''
    seeds near the 8 UR IK branches of the pose q reaches (q: short names, radians), q's own branch first: the
    shoulder flip (pan + pi, lift/elbow/wrist_1 mirrored), the elbow flip (elbow sign, lift and wrist_1 keep the
    wrist's pitch) and the wrist flip (wrist_1 + pi, -wrist_2, wrist_3 + pi; exact for the UR). Approximate except the
    wrist flip: an IK solver seeded with them returns the nearest exact branch. Wrapped into limits.
    '''
    p, l, e, w1, w2, w3 = (q[j] for j in UR_JOINTS)
    seeds = []
    for shoulder in (False, True):
        for elbow in (False, True):
            for wrist in (False, True):
                a = [p, l, e, w1, w2, w3]
                if elbow:
                    a = [a[0], a[1] + a[2], -a[2], a[3] + a[2], a[4], a[5]]
                if shoulder:
                    a = [a[0] + math.pi, -math.pi - a[1], -a[2], -math.pi - a[3], -a[4], a[5]]
                if wrist:
                    a = [a[0], a[1], a[2], a[3] + math.pi, -a[4], a[5] + math.pi]
                seeds.append({j: _wrap_into(v, *limits.get(j, (-2 * math.pi, 2 * math.pi)))
                              for j, v in zip(UR_JOINTS, a)})
    return seeds


def dedupe_branches(solutions, tol=math.radians(2.0)):
    '''solutions (joint dicts) without near duplicates, first kept'''
    kept = []
    for s in solutions:
        if not any(max(abs(s[j] - k[j]) for j in s if j in k) < tol for k in kept):
            kept.append(s)
    return kept


def rank_branches(grid, start, branches, margin, step=math.radians(2.5)):
    '''
    [(branch, goal stretch, path worst, waypoints)] best first: the branches whose goal state is slack before the
    others, then by the stretch of the slack path the grid finds (the straight line's when none), then goal stretch
    '''
    rows = []
    for b in branches:
        goal = grid.stretch(b)
        found = slack_path(grid, start, b, margin, step) if start is not None else ([], goal)
        if found is None:
            rows.append((b, goal, grid.worst_along(start, b, step), None))
        else:
            rows.append((b, goal, found[1], found[0]))
    rows.sort(key=lambda r: (r[1] > margin, r[3] is None, r[2], r[1]))
    return rows
