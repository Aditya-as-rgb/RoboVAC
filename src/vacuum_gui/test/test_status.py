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

"""Rates, real-time factor, localisation judgement and the health lights."""

from vacuum_gui import status


def test_rate_meter_counts_messages_per_second():
    meter = status.RateMeter(window=10.0)
    for i in range(11):
        meter.add(100.0 + i * 0.1)          # 10 Hz
    assert abs(meter.rate(101.0) - 10.0) < 0.5
    assert meter.idle_for(101.5) == 0.5


def test_rate_meter_is_zero_with_a_single_sample_and_drops_stale_ones():
    meter = status.RateMeter(window=2.0)
    assert meter.rate() == 0.0
    meter.add(100.0)
    assert meter.rate(100.5) == 0.0
    for i in range(5):
        meter.add(100.0 + i * 0.1)
    assert meter.rate(100.5) > 0.0
    assert meter.rate(200.0) == 0.0         # everything aged out of the window


def test_real_time_factor_smooths_towards_the_sample_ratio():
    rtf = status.RealTimeFactor(alpha=0.5)
    rtf.add(0.0, 0.0)
    rtf.add(1.0, 2.0)                       # half real time
    first = rtf.value
    assert 0.4 < first < 0.6
    rtf.add(2.0, 4.0)
    assert abs(rtf.value - first) < 1e-9    # still ~0.5, smoothed, not spiking
    rtf.add(3.0, 6.0)
    assert rtf.value < 0.6


def test_localisation_state_thresholds():
    assert status.localisation_state(0.01, 0.01, 0.01, 1.0) == 'localised'
    assert status.localisation_state(0.3, 0.3, 0.01, 1.0) == 'weak'
    assert status.localisation_state(0.01, 0.01, 0.01, 30.0) == 'lost'
    assert status.localisation_state(0.01, 0.01, 0.01, 8.0) == 'unknown'
    assert status.localisation_state(2.0, 0.01, 0.01, 1.0) == 'lost'
    assert status.localisation_state(None, None, None, 1.0) == 'unknown'


def test_health_lights_and_worst_state():
    ok = status.light_for_node('amcl', True)
    assert ok.state == status.OK and ok.color.startswith('#')
    warn = status.light_for_node('controller_server', True, is_lifecycle=False)
    assert warn.state == status.WARN
    fail = status.light_for_node('dirt_detector', False)
    assert fail.state == status.FAIL and 'not running' in fail.detail
    assert status.worst_state([ok, warn, fail]) == status.FAIL
    assert status.worst_state([ok, warn]) == status.WARN
    assert status.worst_state([ok]) == status.OK


def test_topic_light_uses_rate_and_staleness():
    stale = status.light_for_topic('/scan', 0.0, stale_age=5.0, stale_limit=2.0)
    assert stale.state == status.FAIL and 'no message' in stale.detail
    slow = status.light_for_topic('/camera/image', 3.0, expected_min=5.0)
    assert slow.state == status.WARN and 'Hz' in slow.detail
    good = status.light_for_topic('/odom', 30.0)
    assert good.state == status.OK


def test_tf_health_reports_missing_frames():
    bad = status.tf_health(False, missing=['map->odom'])
    assert bad.state == status.FAIL and 'map->odom' in bad.detail
    assert status.tf_health(True).state == status.OK


def test_health_summary_names_the_failures():
    items = [status.light_for_node('amcl', True), status.light_for_node('amcl2', False)]
    state, text = status.summarise_health(items)
    assert state == status.FAIL and 'amcl2' in text
    assert status.summarise_health([])[0] == status.UNKNOWN


def test_formatters():
    assert status.fmt_rate(0.0) == '--'
    assert status.fmt_rate(12.34) == '12.3 Hz'
    assert status.fmt_age(float('inf')) == 'never'
    assert status.fmt_age(0.05) == '50 ms'
    assert status.fmt_age(130.0) == '2 min'
    assert status.fmt_clock(3725) == '01:02:05'
