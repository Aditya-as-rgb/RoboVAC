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

"""Settings merge/save, presets and the layout file."""

import json

from vacuum_gui import settings
from vacuum_gui.commands import StackConfig


def test_defaults_cover_every_stack_field():
    data = settings.load(path='/nonexistent/settings.json')
    assert set(data['stack']) == set(StackConfig().to_dict())
    assert data['ui']['theme'] == 'system'
    assert 'show_scan' in data['view']


def test_load_merges_partial_files_and_ignores_unknown_keys(tmp_path):
    path = tmp_path / 'settings.json'
    path.write_text(json.dumps({'ui': {'theme': 'dark'},
                                'stack': {'dirt': 7, 'headless': 'false'},
                                'nonsense': 1}))
    data = settings.load(path)
    assert data['ui']['theme'] == 'dark'
    assert data['ui']['help_seen'] is False           # untouched default survives
    assert data['stack']['dirt'] == 7 and data['stack']['headless'] is False
    assert 'nonsense' not in data


def test_load_survives_a_corrupt_file(tmp_path):
    path = tmp_path / 'settings.json'
    path.write_text('{not json')
    data = settings.load(path)
    assert data['stack']['world'] == 'living_room.world'


def test_save_writes_json_and_creates_the_folder(tmp_path):
    path = tmp_path / 'sub' / 'settings.json'
    data = settings.load(path)
    data['ui']['theme'] = 'light'
    settings.save(data, path)
    assert json.loads(path.read_text())['ui']['theme'] == 'light'
    assert not (tmp_path / 'sub' / 'settings.json.tmp').exists()


def test_presets_have_the_five_documented_configurations():
    presets = settings.load_presets(path='/nonexistent/presets.json')
    assert set(presets) == {'Demo (localise + coverage + detector)', 'Mapping',
                            'Dataset collection (headless)', 'Vision test', 'Hardware later'}
    demo = presets['Demo (localise + coverage + detector)']
    assert demo['stack']['detector'] is True and demo['stack']['slam'] is False
    assert demo['stack']['headless'] is True
    dataset = presets['Dataset collection (headless)']['stack']
    assert dataset['dirt_clean'] is False and dataset['collect_dataset'] is True
    assert dataset['dirt_randomize'] is True
    mapping = presets['Mapping']['stack']
    assert mapping['slam'] is True and mapping['coverage'] is False


def test_user_presets_override_and_are_saved(tmp_path):
    path = tmp_path / 'presets.json'
    presets = settings.load_presets(path)
    presets['Mine'] = settings.custom_preset('Mine', StackConfig(dirt=3, headless=False),
                                            notes='test')['Mine']
    settings.save_presets(presets, path)
    reloaded = settings.load_presets(path)
    assert reloaded['Mine']['stack']['dirt'] == 3
    assert reloaded['Mine']['stack']['headless'] is False
    assert 'Mapping' in reloaded


def test_layout_round_trip(tmp_path):
    path = tmp_path / 'layout.json'
    assert settings.load_layout(path) == {}
    settings.save_layout('geom', 'state', path)
    assert settings.load_layout(path) == {'geometry': 'geom', 'state': 'state'}
    path.write_text('[]')
    assert settings.load_layout(path) == {}
