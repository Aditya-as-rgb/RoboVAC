from vacuum_vision.tracker import SpotTracker


def tr(**kw):
    kw.setdefault('min_hits', 3)
    kw.setdefault('clear_misses', 3)
    return SpotTracker(**kw)


def test_repeated_sightings_merge_and_confirm():
    t = tr()
    for i in range(3):
        t.update([(1.00 + 0.01 * i, 2.0, 'crumbs', 0.6)], i * 0.2)
    assert len(t.spots) == 1 and len(t.confirmed()) == 1
    s = t.confirmed()[0]
    assert abs(s.x - 1.01) < 0.02 and s.hits == 3 and s.kind == 'crumbs'


def test_far_sightings_stay_separate():
    t = tr()
    t.update([(0.0, 0.0, 'crumbs', 0.5), (0.5, 0.0, 'dust', 0.5)], 0.0)
    assert len(t.spots) == 2


def test_two_boxes_one_patch_count_once_per_frame():
    t = tr()
    t.update([(1.0, 1.0, 'spill', 0.9), (1.02, 1.0, 'spill', 0.5)], 0.0)
    assert len(t.spots) == 1 and t.spots[0].hits == 1


def test_one_off_false_positive_expires():
    t = tr(unconfirmed_ttl=2.0)
    t.update([(3.0, 3.0, 'hair', 0.3)], 0.0)
    t.update([], 1.0)
    assert len(t.spots) == 1
    t.update([], 3.0)
    assert t.spots == []


def test_kind_is_confidence_weighted_vote():
    t = tr()
    t.update([(1, 1, 'dust', 0.3)], 0.0)
    t.update([(1, 1, 'crumbs', 0.8)], 0.2)
    t.update([(1, 1, 'dust', 0.3)], 0.4)
    assert t.spots[0].kind == 'crumbs'


def confirmed_spot(t):
    for i in range(3):
        t.update([(1.0, 1.0, 'crumbs', 0.7)], i * 0.2)
    assert len(t.confirmed()) == 1


def test_spot_cleared_after_enough_misses_in_view():
    t = tr()
    confirmed_spot(t)
    assert t.update([], 1.0, in_view=lambda x, y: True) == []
    assert t.update([], 1.2, in_view=lambda x, y: True) == []
    gone = t.update([], 1.4, in_view=lambda x, y: True)
    assert len(gone) == 1 and t.spots == []


def test_spot_kept_when_camera_is_not_looking():
    t = tr()
    confirmed_spot(t)
    for k in range(10):
        assert t.update([], 1.0 + k, in_view=lambda x, y: False) == []
    assert len(t.confirmed()) == 1


def test_seen_again_resets_misses():
    t = tr()
    confirmed_spot(t)
    t.update([], 1.0, in_view=lambda x, y: True)
    t.update([], 1.2, in_view=lambda x, y: True)
    t.update([(1.0, 1.0, 'crumbs', 0.7)], 1.4, in_view=lambda x, y: True)
    assert t.confirmed()[0].misses == 0
    assert t.update([], 1.6, in_view=lambda x, y: True) == []
