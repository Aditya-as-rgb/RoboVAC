"""Clean-up passes and timings in the mission runner, with a fake robot (no ROS)."""

from test_mission import FakeRobot, make_plan, runner
from vacuum_coverage import core


class Extra:
    def __init__(self, spots):
        self.pieces = [core.Piece('spot', [(x, y, 0.0)]) for x, y in spots]


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        self.t += 1.0
        return self.t


def test_cleanup_pass_runs_after_the_main_pass():
    robot = FakeRobot(pose=(0.0, 0.0, 0.0))
    asked = []

    def reclean(pass_no):
        asked.append(pass_no)
        return Extra([(5.0, 5.0), (6.0, 5.0)]) if pass_no == 1 else None

    ended = []
    res = runner(robot, make_plan(), reclean=reclean, pass_done=ended.append).run()
    assert asked == [1, 2][:len(asked)] and asked[0] == 1
    assert [p.kind for p in res.pieces] == ['wall', 'lane', 'lane', 'spot', 'spot']
    assert [p.pass_no for p in res.pieces] == [0, 0, 0, 1, 1]
    assert ended == [0, 1]
    assert res.passes == 2 and res.done == 5


def test_spots_are_driven_to_not_followed():
    robot = FakeRobot(pose=(0.0, 0.0, 0.0))
    runner(robot, make_plan(1), reclean=lambda n: Extra([(5.0, 5.0)]) if n == 1 else None).run()
    assert ('nav', 5.0, 5.0) in robot.calls
    assert sum(1 for c in robot.calls if c[0] == 'follow') == 1      # only the main piece


def test_spot_needs_a_tight_arrival_not_the_loose_transit_tolerance():
    robot = FakeRobot(pose=(5.2, 5.0, 0.0))             # 0.2 m from the spot: close, not there
    runner(robot, make_plan(1), reclean=lambda n: Extra([(5.0, 5.0)]) if n == 1 else None).run()
    assert ('nav', 5.0, 5.0) in robot.calls


def test_unreachable_spot_is_skipped_not_fatal():
    robot = FakeRobot(pose=(0.0, 0.0, 0.0))
    state = {'main_done': False}

    def reclean(n):
        if n == 1:
            robot.fail_navigate = 99
            return Extra([(9.0, 9.0)])
        return None

    res = runner(robot, make_plan(1), reclean=reclean, max_retries=1).run()
    assert res.pieces[-1].kind == 'spot' and res.pieces[-1].status == 'skipped'
    assert res.skipped == 1 and not res.cancelled
    assert state['main_done'] is False


def test_passes_are_limited():
    robot = FakeRobot()
    calls = []

    def reclean(n):
        calls.append(n)
        return Extra([(2.0 * n, 2.0)])

    res = runner(robot, make_plan(1), reclean=reclean, reclean_passes=2).run()
    assert calls == [1, 2]
    assert res.passes == 3


def test_no_cleanup_pass_after_cancel():
    robot = FakeRobot()
    r = None

    def reclean(n):
        raise AssertionError('must not be asked after a cancel')

    r = runner(robot, make_plan(2), reclean=reclean)
    r.cancel()
    res = r.run()
    assert res.cancelled


def test_cleanup_is_off_without_a_callback():
    res = runner(FakeRobot(), make_plan()).run()
    assert res.passes == 1 and res.done == 3


def test_piece_times_come_from_the_clock():
    res = runner(FakeRobot(), make_plan(2), clock=Clock()).run()
    for p in res.pieces:
        assert p.t_end > p.t_start > 0
    assert res.pieces[1].t_start >= res.pieces[0].t_end
