"""Themes, button feedback, toasts and the background helper."""
import time

import pytest
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton

from vacuum_gui import feedback, theme
from vacuum_gui.background import Background
from vacuum_gui.launch_manager import BLOCKING_KINDS, LEFTOVER_KINDS
from vacuum_gui.toast import ToastManager


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


def pump(app, ms=50):
    end = time.monotonic() + ms / 1000
    while time.monotonic() < end:
        app.processEvents()


def test_theme_registry_and_legacy_names():
    keys = [k for k, _ in theme.theme_choices()]
    assert len(set(keys)) == len(keys) >= 8 and theme.SYSTEM in keys
    assert theme.normalize('dark') == 'graphite'
    assert theme.normalize('light') == 'daylight'
    assert theme.normalize('no-such-theme') == theme.SYSTEM


@pytest.mark.parametrize('key', [k for k in theme.THEMES])
def test_every_theme_applies(app, key):
    assert theme.apply(app, key) == key
    c = theme.colors(key)
    assert c['hover'] != c['pressed']


def test_blocking_kinds_are_subset():
    assert set(BLOCKING_KINDS) <= set(LEFTOVER_KINDS)
    assert 'rviz2' not in BLOCKING_KINDS


def test_busy_button_cycle(app):
    b = QPushButton('Start')
    feedback.begin(b, 'Starting')
    assert feedback.is_busy(b) and not b.isEnabled() and 'Starting' in b.text()
    feedback.set_text(b, 'Launching')
    assert 'Launching' in b.text()
    feedback.end(b, 'ok', enable=True)
    assert not feedback.is_busy(b) and b.isEnabled() and b.text() == 'Start'
    assert b.property('result') == 'ok'


def test_toast_lifecycle(app):
    w = QMainWindow()
    w.resize(800, 600)
    w.show()
    tm = ToastManager(w)
    t = tm.busy('Working')
    assert t.level == 'busy'
    tm.finish(t, 'Done', 'ok')
    assert t.level == 'ok' and len(tm.toasts) == 1
    for i in range(10):
        tm.show('msg %d' % i, 'info')
    assert len(tm.toasts) <= 5


def test_background_runs_off_thread_and_calls_back(app):
    import threading
    bg, got = Background(), []
    main = threading.get_ident()
    seen = []
    bg.run(lambda: seen.append(threading.get_ident()) or 42,
           lambda r, e: got.append((r, e, threading.get_ident())))
    for _ in range(40):
        pump(app, 25)
        if got:
            break
    assert got[0][:2] == (42, None) and got[0][2] == main and seen[0] != main


def test_background_reports_errors(app):
    bg, got = Background(), []

    def boom():
        raise ValueError('x')
    bg.run(boom, lambda r, e: got.append(e))
    for _ in range(40):
        pump(app, 25)
        if got:
            break
    assert isinstance(got[0], ValueError)
