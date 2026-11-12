# Copyright 2026 Aditya Choure
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Rates, real-time factor, localisation quality and the health lights.

Pure functions over plain numbers: the ROS thread only feeds them samples, and the
tests can drive them with synthetic sequences.
"""

from dataclasses import dataclass, field
from collections import deque
import math
import time

OK, WARN, FAIL, UNKNOWN = 'ok', 'warn', 'fail', 'unknown'

LIGHT_COLORS = {OK: '#2f9e44', WARN: '#e8a33d', FAIL: '#e03131', UNKNOWN: '#868e96'}


class RateMeter:
    """Messages per second over a sliding window (wall-clock seconds)."""

    def __init__(self, window=3.0):
        self.window = float(window)
        self.times = deque()

    def add(self, t=None):
        now = time.monotonic() if t is None else float(t)
        self.times.append(now)
        while self.times and now - self.times[0] > self.window:
            self.times.popleft()
        return now

    def rate(self, now=None):
        if len(self.times) < 2:
            return 0.0
        now = time.monotonic() if now is None else float(now)
        while self.times and now - self.times[0] > self.window:
            self.times.popleft()
        if len(self.times) < 2:
            return 0.0
        span = self.times[-1] - self.times[0]
        if span <= 0.0:
            return 0.0
        return (len(self.times) - 1) / span

    def idle_for(self, now=None):
        if not self.times:
            return float('inf')
        now = time.monotonic() if now is None else float(now)
        return max(0.0, now - self.times[-1])


class RealTimeFactor:
    """Sim seconds per wall second, smoothed, from /clock samples."""

    def __init__(self, alpha=0.2):
        self.alpha = float(alpha)
        self.value = 0.0
        self._last = None
        self._last_wall = None

    def add(self, sim_time, wall_time=None):
        wall = time.monotonic() if wall_time is None else float(wall_time)
        last_sim = self._last
        last_wall = self._last_wall
        if last_sim is not None and last_wall is not None:
            ds = float(sim_time) - last_sim
            dw = wall - last_wall
            if dw > 0.0 and 0.0 <= ds < 5.0 * dw + 1.0:
                sample = ds / dw
                self.value = sample if self.value == 0.0 else (
                    (1.0 - self.alpha) * self.value + self.alpha * sample)
        self._last = float(sim_time)
        self._last_wall = wall
        return self.value


def localisation_state(cov_xx, cov_yy, cov_yaw, age_s, localised_age=5.0, lost_age=15.0):
    """AMCL covariance -> ``localised`` / ``weak`` / ``lost`` / ``unknown``.

    Thresholds are deliberately loose: AMCL in a big open room sits around 0.02 m²
    and 0.01 rad² even when it is fine. ``age_s`` is how old the last /amcl_pose is.
    """
    if age_s is None or not math.isfinite(age_s) or age_s > lost_age:
        return 'lost'
    if age_s > localised_age:
        return 'unknown'
    if cov_xx is None or cov_yy is None:
        return 'unknown'
    var = max(float(cov_xx), float(cov_yy))
    yaw = float(cov_yaw) if cov_yaw is not None else 0.0
    if var > 1.0 or yaw > 0.5:
        return 'lost'
    if var > 0.2 or yaw > 0.15:
        return 'weak'
    return 'localised'


@dataclass
class HealthItem:
    key: str
    label: str
    state: str = UNKNOWN
    detail: str = ''
    hint: str = ''

    @property
    def color(self):
        return LIGHT_COLORS.get(self.state, LIGHT_COLORS[UNKNOWN])


def light_for_node(name, present, expected=True, is_lifecycle=None):
    """A node light: green when it is there, amber when it is there but inactive."""
    if not present:
        return HealthItem(name, '/' + name.lstrip('/'), FAIL,
                          'node not running',
                          'start the stack, or check the log tab for a crash')
    if is_lifecycle is False:
        return HealthItem(name, '/' + name.lstrip('/'), WARN,
                          'node up but not active',
                          'wait for Nav2 to activate, or read its log tab')
    return HealthItem(name, '/' + name.lstrip('/'), OK, 'running')


def light_for_topic(name, rate, expected_min=1.0, stale_age=None, stale_limit=2.0):
    if stale_age is not None and stale_age > stale_limit:
        return HealthItem('topic:' + name, name, FAIL,
                          'no message for %.1f s' % stale_age,
                          'publisher down, or the sim is paused: check /clock and the log tabs')
    if rate < expected_min:
        return HealthItem('topic:' + name, name, WARN,
                          '%.1f Hz (expected >= %.1f)' % (rate, expected_min),
                          'low rate is normal while the scene loads; if it stays low, check the GPU load')
    return HealthItem('topic:' + name, name, OK, '%.1f Hz' % rate)


def tf_health(ok, missing=(), note=''):
    if ok:
        return HealthItem('tf', 'TF chain', OK, note or 'map -> odom -> base_footprint')
    return HealthItem('tf', 'TF chain', FAIL,
                      note or ('missing: ' + ', '.join(missing) if missing else 'no map -> base_footprint'),
                      'AMCL/SLAM publishes map->odom; check the localisation state and /clock')


def worst_state(items):
    order = {FAIL: 3, WARN: 2, UNKNOWN: 1, OK: 0}
    state = OK
    for item in items:
        if order.get(item.state, 0) > order[state]:
            state = item.state
    return state


def summarise_health(items):
    """(state, headline) for the health tab header."""
    failures = [i for i in items if i.state == FAIL]
    warnings = [i for i in items if i.state == WARN]
    if failures:
        return FAIL, '%d failing: %s' % (len(failures), ', '.join(i.label for i in failures[:4]))
    if warnings:
        return WARN, '%d warning(s)' % len(warnings)
    if not items:
        return UNKNOWN, 'nothing to check yet'
    return OK, 'all %d checks green' % len(items)


def fmt_rate(hz):
    return '--' if hz <= 0.0 else ('%.1f Hz' % hz)


def fmt_age(age):
    if age == float('inf'):
        return 'never'
    if age < 1.0:
        return '%.0f ms' % (age * 1000.0)
    if age < 120.0:
        return '%.1f s' % age
    return '%.0f min' % (age / 60.0)


def fmt_clock(seconds):
    seconds = max(0, int(seconds))
    return '%02d:%02d:%02d' % (seconds // 3600, (seconds % 3600) // 60, seconds % 60)
