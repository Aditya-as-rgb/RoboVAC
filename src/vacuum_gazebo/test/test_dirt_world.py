"""Dirt generator tests (no ROS, no Gazebo): pytest src/vacuum_gazebo/test"""

import importlib.util
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SPEC = importlib.util.spec_from_file_location('dirt_world', os.path.join(PKG, 'scripts', 'dirt_world.py'))
dw = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dw)

WORLD = os.path.join(PKG, 'worlds', 'living_room.world')
MAP = os.path.join(PKG, 'maps', 'living_room.yaml')
SPAWN = (-2.0, -0.5)


def write_map(tmp_path, grid, res=0.05, origin=(0.0, 0.0)):
    """grid: 2-D array, row 0 = TOP of the picture; 254 free, 0 occupied."""
    h, w = grid.shape
    pgm = tmp_path / 'm.pgm'
    pgm.write_bytes(b'P5\n%d %d\n255\n' % (w, h) + grid.astype(np.uint8).tobytes())
    yml = tmp_path / 'm.yaml'
    yml.write_text('image: m.pgm\nmode: trinary\nresolution: %s\norigin: [%s, %s, 0]\nnegate: 0\n'
                   'occupied_thresh: 0.65\nfree_thresh: 0.196\n' % (res, origin[0], origin[1]))
    return str(yml)


def test_living_room_map_loads_with_row0_at_the_bottom():
    fmap = dw.load_map(MAP)
    assert fmap.shape == (98, 99)
    assert fmap.free.sum() > 3000
    # the spawn point is free floor
    r, c = fmap.world_to_cell(*SPAWN)
    assert fmap.free[r, c]
    # the room is walled in: the border rows/cols are not free
    assert not fmap.free[0].any() or not fmap.free[-1].any()


def test_same_seed_same_layout_different_seed_differs():
    fmap = dw.load_map(MAP)
    a = dw.scatter_dirt(fmap, 12, seed=5)
    b = dw.scatter_dirt(fmap, 12, seed=5)
    c = dw.scatter_dirt(fmap, 12, seed=6)
    assert [(p['x'], p['y'], p['kind']) for p in a] == [(p['x'], p['y'], p['kind']) for p in b]
    assert [(p['x'], p['y']) for p in a] != [(p['x'], p['y']) for p in c]


def test_patches_are_on_clean_able_floor_and_spaced_out():
    fmap = dw.load_map(MAP)
    patches = dw.scatter_dirt(fmap, 20, seed=2)
    assert len(patches) == 20
    blocked_xy = np.array([fmap.cell_to_world(r, c) for r, c in np.argwhere(fmap.blocked)])
    for p in patches:
        r, c = fmap.world_to_cell(p['x'], p['y'])
        assert fmap.free[r, c]
        assert np.hypot(blocked_xy[:, 0] - p['x'], blocked_xy[:, 1] - p['y']).min() >= 0.10 - fmap.res
        assert math.hypot(p['x'] - SPAWN[0], p['y'] - SPAWN[1]) > 0.45
    for i, p in enumerate(patches):
        for q in patches[:i]:
            assert math.hypot(p['x'] - q['x'], p['y'] - q['y']) >= 0.45 - 1e-6


def test_edge_fraction_puts_patches_near_walls():
    fmap = dw.load_map(MAP)
    patches = dw.scatter_dirt(fmap, 16, seed=1, edge_fraction=0.25)
    assert sum(p['zone'] == 'edge' for p in patches) == 4
    none = dw.scatter_dirt(fmap, 16, seed=1, edge_fraction=0.0)
    assert all(p['zone'] == 'interior' for p in none)


def test_dirt_never_lands_in_an_unreachable_pocket(tmp_path):
    grid = np.full((60, 60), 254, dtype=np.uint8)
    grid[:2, :] = grid[-2:, :] = grid[:, :2] = grid[:, -2:] = 0
    grid[20:40, 20:22] = grid[20:40, 38:40] = 0           # a closed 20 x 20 cell box in the middle
    grid[20:22, 20:40] = grid[38:40, 20:40] = 0
    yml = write_map(tmp_path, grid)
    fmap = dw.load_map(yml)
    patches = dw.scatter_dirt(fmap, 15, seed=4, spawn=(0.5, 0.5))
    assert patches
    for p in patches:
        inside = 1.0 <= p['x'] <= 2.0 and 1.0 <= p['y'] <= 2.0
        assert not inside, p


def test_too_cluttered_map_raises(tmp_path):
    grid = np.zeros((20, 20), dtype=np.uint8)
    grid[8:12, 8:12] = 254
    with pytest.raises(ValueError):
        dw.scatter_dirt(dw.load_map(write_map(tmp_path, grid)), 5, spawn=(0.5, 0.5))


def test_world_gets_valid_dirt_models_without_collision(tmp_path):
    res = dw.make_world(WORLD, MAP, 10, seed=3, spawn=SPAWN, out_dir=str(tmp_path))
    text = open(res['world']).read()
    original = open(WORLD).read()
    assert text.startswith(original[:original.rfind('</world>')])       # nothing else touched
    root = ET.fromstring(text)
    world = root.find('world')
    assert world.get('name') == 'default'
    dirt = [m for m in world.findall('model') if m.get('name', '').startswith('dirt_')]
    assert [m.get('name') for m in dirt] == ['dirt_%03d' % i for i in range(10)]
    for m in dirt:
        assert m.find('static').text == 'true'
        assert m.find('.//collision') is None
        visuals = m.findall('.//visual')
        assert visuals
        for v in visuals:
            geom = v.find('geometry')
            if geom.find('box') is not None:                  # every dimension of a dirt piece is tiny
                assert float(geom.find('box/size').text.split()[2]) < 0.01
            elif geom.find('sphere') is not None:
                assert float(geom.find('sphere/radius').text) < 0.01
            else:
                assert float(geom.find('cylinder/length').text) < 0.01
    # the original models are all still there
    assert len(world.findall('model')) == len(ET.fromstring(original).find('world').findall('model')) + 10


def test_layout_json_matches_the_world(tmp_path):
    res = dw.make_world(WORLD, MAP, 8, seed=9, spawn=SPAWN, out_dir=str(tmp_path))
    layout = json.load(open(res['layout']))
    assert layout['seed'] == 9 and layout['count'] == 8 and len(layout['patches']) == 8
    assert os.path.basename(res['world']) == 'living_room_dirt8_seed9.world'
    root = ET.fromstring(open(res['world']).read())
    poses = {m.get('name'): [float(v) for v in m.find('pose').text.split()[:2]]
             for m in root.find('world').findall('model') if m.get('name', '').startswith('dirt_')}
    for p in layout['patches']:
        assert poses[p['name']] == pytest.approx([p['x'], p['y']], abs=1e-3)
        assert p['kind'] in dw.KINDS


def test_not_a_world_file_is_rejected(tmp_path):
    bad = tmp_path / 'x.world'
    bad.write_text('<sdf></sdf>')
    with pytest.raises(ValueError):
        dw.make_world(str(bad), MAP, 3, out_dir=str(tmp_path))


def test_zero_patches_is_an_error():
    with pytest.raises(ValueError):
        dw.scatter_dirt(dw.load_map(MAP), 0)


# ------------------------------------------------ realism / scrubbing fields ---

def test_all_four_kinds_appear_with_pieces_of_tiny_size():
    fmap = dw.load_map(MAP)
    patches = dw.scatter_dirt(fmap, 40, seed=7, min_spacing=0.25)
    assert {p['kind'] for p in patches} == set(dw.KINDS)
    for p in patches:
        assert len(p['shapes']) >= 3
        for s in p['shapes']:
            assert 0.0 <= min(s['color']) and max(s['color']) <= 1.0
            assert math.hypot(s['dx'], s['dy']) <= 0.30


def test_scale_makes_patches_bigger_but_keeps_the_layout_positions():
    fmap = dw.load_map(MAP)
    small = dw.scatter_dirt(fmap, 12, seed=2, scale=1.0)
    big = dw.scatter_dirt(fmap, 12, seed=2, scale=2.0)
    assert [(p['x'], p['y'], p['kind']) for p in small] == [(p['x'], p['y'], p['kind']) for p in big]
    assert sum(p['radius_m'] for p in big) > 1.6 * sum(p['radius_m'] for p in small)
    with pytest.raises(ValueError):
        dw.scatter_dirt(fmap, 3, scale=0)


def test_spills_are_always_stubborn_and_need_more_scrubbing():
    fmap = dw.load_map(MAP)
    patches = dw.scatter_dirt(fmap, 40, seed=7, min_spacing=0.25, stubborn_fraction=0.0)
    spills = [p for p in patches if p['kind'] == 'spill']
    others = [p for p in patches if p['kind'] != 'spill']
    assert spills and all(p['stubborn'] and p['scrub_m'] == dw.SCRUB_STUBBORN for p in spills)
    assert all(not p['stubborn'] and p['scrub_m'] == dw.SCRUB_NORMAL for p in others)
    every = dw.scatter_dirt(fmap, 20, seed=7, stubborn_fraction=1.0)
    assert all(p['stubborn'] for p in every)


def test_stubborn_fraction_does_not_move_anything_else():
    fmap = dw.load_map(MAP)
    a = dw.scatter_dirt(fmap, 15, seed=4, stubborn_fraction=0.0)
    b = dw.scatter_dirt(fmap, 15, seed=4, stubborn_fraction=0.5)
    assert [(p['x'], p['y'], p['kind'], p['radius_m']) for p in a] == \
        [(p['x'], p['y'], p['kind'], p['radius_m']) for p in b]


def test_current_layout_is_written_and_matches(tmp_path):
    res = dw.make_world(WORLD, MAP, 6, seed=5, spawn=SPAWN, out_dir=str(tmp_path), scale=1.5)
    cur = json.load(open(os.path.join(str(tmp_path), 'current_layout.json')))
    assert cur == json.load(open(res['layout']))
    assert cur['scale'] == 1.5
    assert set(cur['patches'][0]) == set(dw.LAYOUT_KEYS)


def test_dust_is_translucent_and_spills_glossy_in_the_sdf(tmp_path):
    fmap = dw.load_map(MAP)
    patches = dw.scatter_dirt(fmap, 40, seed=7, min_spacing=0.25)
    dust = next(p for p in patches if p['kind'] == 'dust')
    spill = next(p for p in patches if p['kind'] == 'spill')
    assert '<transparency>' in dw.patch_sdf(dust)
    assert '<specular>0.80 0.80 0.80 1</specular>' in dw.patch_sdf(spill)


def test_pictures_render(tmp_path):
    pytest.importorskip('matplotlib')
    fmap = dw.load_map(MAP)
    patches = dw.scatter_dirt(fmap, 20, seed=1, min_spacing=0.3)
    dw.render_preview(str(tmp_path / 'a.png'), MAP, patches, SPAWN)
    dw.render_closeups(str(tmp_path / 'b.png'), patches)
    assert (tmp_path / 'a.png').stat().st_size > 5000 and (tmp_path / 'b.png').stat().st_size > 5000


def test_layout_has_hull_and_height_for_every_patch(tmp_path):
    res = dw.make_world(WORLD, MAP, 12, seed=2, spawn=SPAWN, out_dir=str(tmp_path))
    for p in res['patches']:
        assert len(p['hull']) >= 3 and p['height_m'] > 0
        assert all(math.hypot(x, y) <= p['radius_m'] + 0.05 for x, y in p['hull'])


def test_randomize_changes_floor_and_light_deterministically(tmp_path):
    a = dw.make_world(WORLD, MAP, 5, seed=4, spawn=SPAWN, out_dir=str(tmp_path / 'a'), randomize=True)
    b = dw.make_world(WORLD, MAP, 5, seed=4, spawn=SPAWN, out_dir=str(tmp_path / 'b'), randomize=True)
    off = dw.make_world(WORLD, MAP, 5, seed=4, spawn=SPAWN, out_dir=str(tmp_path / 'c'))
    ta, tb, to = (open(r['world']).read() for r in (a, b, off))
    assert ta == tb and ta != to
    root = ET.fromstring(ta)
    names = [m.get('name') for m in root.find('world').findall('model')]
    assert 'floor_tint' in names
    assert 'floor_tint' not in to
    env = json.load(open(a['layout']))['environment']
    assert len(env['floor_color']) == 3 and len(env['ambient']) == 3
    assert json.load(open(off['layout']))['environment'] is None
    envs = {tuple(dw.environment_variation(s)['floor_color']) for s in range(1, 20)}
    assert len(envs) > 5
