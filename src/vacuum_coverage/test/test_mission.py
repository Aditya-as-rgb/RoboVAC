"""Mission sequencing tests with a fake robot (no ROS)."""


from vacuum_coverage import core, mission


class FakeRobot:
    def __init__(self, pose=(0.0, 0.0, 0.0), fail_follow=(), fail_navigate=0):
        self.pose = pose
        self.fail_follow = list(fail_follow)      # follow() call numbers (1-based) that abort half way
        self.fail_navigate = fail_navigate        # number of navigate() calls that fail first
        self.calls = []
        self.follow_calls = 0

    def navigate(self, x, y, yaw):
        self.calls.append(('nav', round(x, 2), round(y, 2)))
        if self.fail_navigate > 0:
            self.fail_navigate -= 1
            return False
        self.pose = (x, y, yaw)
        return True

    def follow(self, poses):
        self.follow_calls += 1
        self.calls.append(('follow', len(poses)))
        if self.follow_calls in self.fail_follow:
            mid = poses[len(poses) // 2]
            self.pose = mid
            return False
        self.pose = poses[-1]
        return True

    def get_pose(self):
        return self.pose


def make_plan(n_pieces=3):
    pieces = []
    for k in range(n_pieces):
        poses = [(1.0 * k + 0.1 * i, 1.0 * k, 0.0) for i in range(20)]
        pieces.append(core.Piece('wall' if k == 0 else 'lane', poses))

    class P:
        pass
    p = P()
    p.pieces = pieces
    return p


def runner(robot, plan, **kw):
    return mission.MissionRunner(plan, robot.navigate, robot.follow, robot.get_pose, **kw)


def test_runs_all_pieces_in_order_with_transits():
    robot = FakeRobot(pose=(5.0, 5.0, 0.0))
    res = runner(robot, make_plan()).run()
    assert res.done == 3 and res.skipped == 0 and not res.cancelled
    kinds = [c[0] for c in robot.calls]
    assert kinds.count('follow') == 3
    assert kinds[0] == 'nav'                       # robot started far from piece 1


def test_no_transit_when_already_at_start():
    plan = make_plan(1)
    robot = FakeRobot(pose=plan.pieces[0].poses[0])
    runner(robot, plan).run()
    assert [c[0] for c in robot.calls] == ['follow']


def test_aborted_follow_is_retried_from_nearest_pose():
    plan = make_plan(1)
    robot = FakeRobot(pose=plan.pieces[0].poses[0], fail_follow=[1])
    res = runner(robot, plan).run()
    assert res.pieces[0].status == 'done' and res.pieces[0].attempts == 2
    assert robot.calls[-1] == ('follow', 10)       # second try started at the pose reached (index 10)


def test_gives_up_after_max_retries_and_moves_on():
    plan = make_plan(2)
    robot = FakeRobot(fail_follow=[1, 2, 3])
    res = runner(robot, plan, max_retries=2).run()
    assert res.pieces[0].status == 'skipped'
    assert res.pieces[1].status == 'done'


def test_unreachable_piece_is_skipped():
    plan = make_plan(2)
    robot = FakeRobot(pose=(5.0, 5.0, 0.0), fail_navigate=3)            # first piece: 1 + 2 retries all fail
    res = runner(robot, plan, max_retries=2).run()
    assert res.pieces[0].status == 'skipped' and res.pieces[1].status == 'done'


def test_cancel_stops_between_pieces():
    plan = make_plan(3)
    robot = FakeRobot()
    r = runner(robot, plan)
    orig = robot.follow

    def follow_and_cancel(poses):
        ok = orig(poses)
        r.cancel()
        return ok
    r._follow = follow_and_cancel
    res = r.run()
    assert res.cancelled and len(res.pieces) == 1


def test_trim_from_nearest_keeps_tail():
    poses = [(float(i), 0.0, 0.0) for i in range(10)]
    assert mission.trim_from_nearest(poses, (6.2, 0.1))[0][0] == 6.0
    assert len(mission.trim_from_nearest(poses, (9.9, 0.0))) >= 2
