"""ROS-free mission logic: run a coverage plan piece by piece.

The runner only needs three callables, so it can be tested with a fake robot
(see test/test_mission.py) and wired to Nav2 in mission_node.py:

    navigate(x, y, yaw) -> bool      drive to a pose (Nav2 NavigateToPose)
    follow(poses)       -> bool      follow a path exactly (Nav2 FollowPath)
    robot_pose()        -> (x, y, yaw) or None
"""

from dataclasses import dataclass, field
import math
import threading


@dataclass
class PieceResult:
    index: int
    kind: str
    status: str            # 'done' | 'skipped' | 'cancelled'
    attempts: int = 0


@dataclass
class MissionResult:
    pieces: list = field(default_factory=list)
    cancelled: bool = False

    @property
    def done(self):
        return sum(1 for p in self.pieces if p.status == 'done')

    @property
    def skipped(self):
        return sum(1 for p in self.pieces if p.status == 'skipped')


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def trim_from_nearest(poses, xy, keep_min=2):
    """Drop the poses before the one nearest to ``xy`` (used to retry after an abort)."""
    if len(poses) <= keep_min:
        return list(poses)
    i = min(range(len(poses)), key=lambda k: _dist(poses[k], xy))
    i = min(i, len(poses) - keep_min)
    return list(poses[i:])


class MissionRunner:
    def __init__(self, plan, navigate, follow, robot_pose, log=None, status=None,
                 max_retries=2, transit_tolerance=0.35, skip_done_fraction=0.9):
        self.plan = plan
        self._navigate = navigate
        self._follow = follow
        self._robot_pose = robot_pose
        self._log = log or (lambda msg: None)
        self._status = status or (lambda msg: None)
        self.max_retries = max_retries
        self.transit_tolerance = transit_tolerance
        self.skip_done_fraction = skip_done_fraction
        self._cancel = threading.Event()

    def cancel(self):
        self._cancel.set()

    @property
    def cancelled(self):
        return self._cancel.is_set()

    def run(self):
        result = MissionResult()
        total = len(self.plan.pieces)
        for idx, piece in enumerate(self.plan.pieces):
            if self.cancelled:
                result.cancelled = True
                break
            self._status('piece %d/%d (%s)' % (idx + 1, total, piece.kind))
            res = self._run_piece(idx, piece)
            result.pieces.append(res)
            if res.status == 'cancelled':
                result.cancelled = True
                break
        self._status('cancelled' if result.cancelled else
                     'finished: %d done, %d skipped' % (result.done, result.skipped))
        return result

    # ------------------------------------------------------------------

    def _transit(self, target):
        """Bring the robot to the start of a piece; True when it is there (or close enough)."""
        for attempt in range(self.max_retries + 1):
            if self.cancelled:
                return False
            pose = self._robot_pose()
            if pose is not None and _dist(pose, target) <= self.transit_tolerance:
                return True
            self._log('transit to (%.2f, %.2f), try %d' % (target[0], target[1], attempt + 1))
            if self._navigate(target[0], target[1], target[2]):
                return True
        return False

    def _run_piece(self, idx, piece):
        poses = list(piece.poses)
        res = PieceResult(idx, piece.kind, 'skipped')
        if not self._transit(poses[0]):
            if self.cancelled:
                res.status = 'cancelled'
            else:
                self._log('piece %d: could not reach its start, skipped' % (idx + 1))
            return res
        for attempt in range(self.max_retries + 1):
            if self.cancelled:
                res.status = 'cancelled'
                return res
            res.attempts = attempt + 1
            if self._follow(poses):
                res.status = 'done'
                return res
            if self.cancelled:
                res.status = 'cancelled'
                return res
            pose = self._robot_pose()
            self._log('piece %d: path aborted (try %d)' % (idx + 1, attempt + 1))
            if pose is not None:
                poses = trim_from_nearest(poses, pose)
                # little left to do: count it as done
                if len(poses) < max(2, int(len(piece.poses) * (1.0 - self.skip_done_fraction))):
                    res.status = 'done'
                    return res
                if _dist(pose, poses[0]) > self.transit_tolerance and not self._transit(poses[0]):
                    break
        self._log('piece %d: giving up after %d tries' % (idx + 1, res.attempts))
        return res
