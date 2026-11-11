"""ROS-free mission logic: run a coverage plan piece by piece.

The runner only needs three callables, so it can be tested with a fake robot
(see test/test_mission.py) and wired to Nav2 in mission_node.py:

    navigate(x, y, yaw) -> bool      drive to a pose (Nav2 NavigateToPose)
    follow(poses)       -> bool      follow a path exactly (Nav2 FollowPath)
    robot_pose()        -> (x, y, yaw) or None

Optional extras:

    reclean(pass_no) -> plan or None   called after the main pass; returns a plan whose
                                       pieces (kind 'spot') cover what is still dirty
    pass_done(pass_no)                 called when a pass (0 = main) has finished
    clock() -> seconds                 time source for the piece timings
"""

from dataclasses import dataclass, field
import math
import threading
import time


@dataclass
class PieceResult:
    index: int
    kind: str
    status: str            # 'done' | 'skipped' | 'cancelled'
    attempts: int = 0
    pass_no: int = 0       # 0 = main pass, 1.. = clean-up passes
    t_start: float = 0.0   # clock() when the piece started / ended
    t_end: float = 0.0


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

    @property
    def passes(self):
        """Number of passes that ran (the main pass counts as one)."""
        return len({p.pass_no for p in self.pieces})


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
                 max_retries=2, transit_tolerance=0.35, skip_done_fraction=0.9,
                 reclean=None, reclean_passes=2, pass_done=None, clock=None,
                 spot_tolerance=0.10):
        self.plan = plan
        self._navigate = navigate
        self._follow = follow
        self._robot_pose = robot_pose
        self._log = log or (lambda msg: None)
        self._status = status or (lambda msg: None)
        self.max_retries = max_retries
        self.transit_tolerance = transit_tolerance
        self.skip_done_fraction = skip_done_fraction
        self._reclean = reclean
        self.reclean_passes = reclean_passes
        self._pass_done = pass_done or (lambda pass_no: None)
        self._clock = clock or time.monotonic
        self.spot_tolerance = spot_tolerance
        self._cancel = threading.Event()

    def cancel(self):
        self._cancel.set()

    @property
    def cancelled(self):
        return self._cancel.is_set()

    def run(self):
        result = MissionResult()
        self._run_pass(self.plan, 0, result)
        self._pass_done(0)
        if self._reclean is not None and not result.cancelled:
            for pass_no in range(1, self.reclean_passes + 1):
                extra = self._reclean(pass_no)
                if extra is None or not extra.pieces:
                    self._log('clean-up pass %d: nothing left to clean' % pass_no)
                    break
                self._log('clean-up pass %d: %d spots' % (pass_no, len(extra.pieces)))
                self._run_pass(extra, pass_no, result)
                self._pass_done(pass_no)
                if result.cancelled:
                    break
        self._status('cancelled' if result.cancelled else
                     'finished: %d done, %d skipped' % (result.done, result.skipped))
        return result

    def _run_pass(self, plan, pass_no, result):
        total = len(plan.pieces)
        for idx, piece in enumerate(plan.pieces):
            if self.cancelled:
                result.cancelled = True
                break
            label = 'piece' if pass_no == 0 else 'clean-up %d: spot' % pass_no
            self._status('%s %d/%d (%s)' % (label, idx + 1, total, piece.kind))
            t0 = self._clock()
            res = self._run_piece(idx, piece)
            res.pass_no = pass_no
            res.t_start, res.t_end = t0, self._clock()
            result.pieces.append(res)
            if res.status == 'cancelled':
                result.cancelled = True
                break

    # ------------------------------------------------------------------

    def _transit(self, target, tolerance=None):
        """Bring the robot to the start of a piece; True when it is there (or close enough)."""
        tol = self.transit_tolerance if tolerance is None else tolerance
        for attempt in range(self.max_retries + 1):
            if self.cancelled:
                return False
            pose = self._robot_pose()
            if pose is not None and _dist(pose, target) <= tol:
                return True
            self._log('transit to (%.2f, %.2f), try %d' % (target[0], target[1], attempt + 1))
            if self._navigate(target[0], target[1], target[2]):
                return True
        return False

    def _run_spot(self, idx, piece):
        """A spot is just a place to drive to; the trip there is what cleans."""
        res = PieceResult(idx, piece.kind, 'skipped')
        res.attempts = 1
        if self._transit(piece.poses[0], self.spot_tolerance):
            res.status = 'done'
        elif self.cancelled:
            res.status = 'cancelled'
        else:
            self._log('spot %d: could not reach it, skipped' % (idx + 1))
        return res

    def _run_piece(self, idx, piece):
        if piece.kind == 'spot':
            return self._run_spot(idx, piece)
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
