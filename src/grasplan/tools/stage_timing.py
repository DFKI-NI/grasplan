'''
Where the time of a pick, place or insert goes: one summary line per action, so every speed-up can be
measured with the same numbers. Log only: no behaviour changes, no flag.

    TIMING pick banana ok=1 total=31.42 view=12.50 filters=1.50 plan=1.02 plan/guard_replay=5.00 plan/guard_replay#=3 ...

``begin(kind, target)`` starts an action, ``stage(name)`` (context manager) and ``@timed(name)`` (method decorator) time a
block; stages nest and are named by their path (``plan/guard_replay``); a stage that runs several times is summed and gets
its count in ``<path>#``. ``end(ok)`` logs the line: ``total`` is the wall time of the action, the top-level stages follow
in the order they first ran, ``other`` is the part of ``total`` that no top-level stage covers. Outside an action
everything is a no-op. Not thread safe on purpose: the action server thread runs the whole goal.
'''
import functools
import time
from contextlib import contextmanager


class StageTimer:
    def __init__(self, log=None, clock=time.monotonic):
        self._log = log
        self._clock = clock
        self.action = None   # (kind, target, start) while an action runs
        self.totals = {}     # path -> seconds, in the order first seen
        self.counts = {}     # path -> how often it ran
        self._open = []      # (name, start) of the stages running now
        self.last_ok = False   # result of the last goal, set by watch_server

    def _write(self, line):
        if self._log is None:
            import rospy
            rospy.loginfo(line)
        else:
            self._log(line)

    def begin(self, kind, target=''):
        if self.action is not None:   # the previous goal ended without end(): log what it had
            self.end(False)
        self.action = (kind, str(target), self._clock())
        self.totals, self.counts, self._open = {}, {}, []

    def open(self, name):
        '''start a stage that closes with close() (or with end()), for code that cannot be wrapped in a with block'''
        if self.action is not None:
            self.totals.setdefault('/'.join([n for n, _ in self._open] + [name]), 0.0)   # keeps the start order
            self._open.append((name, self._clock()))

    def close(self):
        if self.action is None or not self._open:
            return
        name, start = self._open.pop()
        path = '/'.join([n for n, _ in self._open] + [name])
        self.totals[path] = self.totals.get(path, 0.0) + self._clock() - start
        self.counts[path] = self.counts.get(path, 0) + 1

    @contextmanager
    def stage(self, name):
        self.open(name)
        depth = len(self._open)
        try:
            yield
        finally:
            while len(self._open) > depth:   # a stage opened inside and never closed
                self.close()
            self.close()

    def end(self, ok):
        if self.action is None:
            return None
        while self._open:
            self.close()
        kind, target, start = self.action
        self.action = None
        total = self._clock() - start
        top = sum(seconds for path, seconds in self.totals.items() if '/' not in path)
        parts = [f'{path}={seconds:.2f}' + (f' {path}#={self.counts[path]}' if self.counts.get(path, 0) > 1 else '')
                 for path, seconds in self.totals.items()]
        line = (f'TIMING {kind} {target or "-"} ok={int(bool(ok))} total={total:.2f} ' + ' '.join(parts)
                + f' other={max(0.0, total - top):.2f}')
        self._write(line)
        return line


TIMER = StageTimer()


def timed(name):
    '''method decorator: the call is one stage of the running action'''
    def wrap(function):
        @functools.wraps(function)
        def inner(*args, **kwargs):
            with TIMER.stage(name):
                return function(*args, **kwargs)
        return inner
    return wrap


def action(kind, target):
    '''decorator of an action callback ``callback(self, goal)``: the call is one timed action; target(goal) names it. The
    result (ok=) is what the callback sets on its server (see watch_server), failed when it sets none.'''
    def wrap(function):
        @functools.wraps(function)
        def inner(self, goal):
            TIMER.begin(kind, target(goal))
            TIMER.last_ok = False
            try:
                return function(self, goal)
            finally:
                TIMER.end(TIMER.last_ok)
        return inner
    return wrap


def watch_server(server):
    '''let TIMER see whether the goals of this actionlib.SimpleActionServer succeed (set_succeeded) or not'''
    for name, ok in (('set_succeeded', True), ('set_aborted', False), ('set_preempted', False)):
        original = getattr(server, name)

        def wrapped(*args, _original=original, _ok=ok, **kwargs):
            TIMER.last_ok = _ok
            return _original(*args, **kwargs)
        setattr(server, name, wrapped)
