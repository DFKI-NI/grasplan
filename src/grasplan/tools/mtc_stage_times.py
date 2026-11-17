'''
Where the time of one MTC plan goes, for the TIMING line and one log line per plan. Log only: no behaviour change,
no flag.

MTC's introspection (on by default) publishes, from the C++ side of the Python bindings, the stage tree of the task on
``<node>_cpp/description`` and the compute time of every stage on ``<node>_cpp/statistics`` after each compute step.
StageTimes listens to both and, after a plan, gives the compute time per leaf stage. The 'current state' stage is the
scene fetch (it asks move_group for the whole planning scene, octomap included), so a plan splits into

    init     task.init(): stage setup, planner plugin
    scene    compute time of the 'current state' stage (part of task.plan())
    compute  the rest of task.plan(): IK, Cartesian and OMPL stages
    finish   solution to message, scene size, publish of the chosen solution

and the cable check (``plan/guard_check``) keeps its own key. scene_size() reads the size of the planning scene a
solution starts from (octomap bytes, world and attached objects) out of its message.
'''
import threading
import time

SCENE_STAGE = 'current state'


class StageTimes:
    def __init__(self, base=None, subscribe=True, clock=time.monotonic):
        '''base: topic namespace of the C++ side (default: this node's name + _cpp)'''
        self._clock = clock
        self._lock = threading.Lock()
        self.names = {}        # task_id -> {stage id: (name, parent id)}
        self.latest = None     # (receipt time, TaskStatistics)
        self.base = base
        if subscribe:
            import rospy
            from moveit_task_constructor_msgs.msg import TaskDescription, TaskStatistics
            self.base = base or rospy.get_name() + '_cpp'
            rospy.Subscriber(self.base + '/description', TaskDescription, self.on_description, queue_size=2)
            rospy.Subscriber(self.base + '/statistics', TaskStatistics, self.on_statistics, queue_size=5)

    def on_description(self, msg):
        with self._lock:
            self.names[msg.task_id] = {s.id: (s.name, s.parent_id) for s in msg.stages}

    def on_statistics(self, msg):
        with self._lock:
            self.latest = (self._clock(), msg)

    def latest_since(self, since, wait=0.01, sleep=time.sleep):
        '''the newest statistics received after since (the start of the plan), None when none came. The scene fetch
        is the first stage to compute, so its time is final in any message of the plan; the newest message may miss the
        last compute step of the other stages (the slowest-stage text only). Waits up to wait seconds only when none of
        this plan came yet and the topic ever delivered: a missing topic costs no time.'''
        end = self._clock() + wait
        while True:
            with self._lock:
                latest = self.latest
            if latest is not None and latest[0] >= since:
                return latest[1]
            if latest is None or self._clock() >= end:
                return None
            sleep(0.002)

    def leaf_times(self, stats):
        '''{stage path: compute seconds} of the leaf stages of stats; None without the matching stage tree'''
        if stats is None:
            return None
        with self._lock:
            tree = self.names.get(stats.task_id)
        if not tree:
            return None
        parents = {parent for _, parent in tree.values()}
        out = {}
        for s in stats.stages:
            if s.id not in tree or s.id in parents:
                continue
            path, node, seen = [], s.id, set()
            while node in tree and node not in seen:
                seen.add(node)
                name, parent = tree[node]
                path.append(name)
                if parent == node:
                    break
                node = parent
            # drop the task's own root container (its name is the task's), keep the rest outermost first
            path = list(reversed(path))[1:] or path[:1]
            out['/'.join(path)] = out.get('/'.join(path), 0.0) + float(s.total_compute_time)
        return out


def scene_seconds(leaves):
    '''compute time of the scene fetch stage among the leaf times (None when unknown)'''
    if leaves is None:
        return None
    hits = [v for k, v in leaves.items() if k.split('/')[-1] == SCENE_STAGE]
    return sum(hits) if hits else None


def slowest(leaves, count=4):
    '''text of the count slowest leaf stages, e.g. "grasp/compute ik 1.20 s, move to pregrasp 0.40 s"'''
    if not leaves:
        return 'n/a'
    top = sorted(leaves.items(), key=lambda kv: -kv[1])[:count]
    return ', '.join(f'{k} {v:.2f} s' for k, v in top)


def scene_size(solution_msg):
    '''(octomap bytes, world objects, attached objects) of the scene a solution message starts from; None if unreadable'''
    try:
        scene = solution_msg.start_scene
        octo = len(scene.world.octomap.octomap.data)
        return octo, len(scene.world.collision_objects), len(scene.robot_state.attached_collision_objects)
    except AttributeError:
        return None


def size_text(size):
    if size is None:
        return 'scene size n/a'
    octo, world, attached = size
    return f'scene: octomap {octo} bytes, {world} world objects, {attached} attached'
