#!/usr/bin/env python3
"""Scatter visual-only dirt on the floor of a Gazebo world.

Works with any world + the map of that world: dirt is placed only on floor the robot can
actually clean (reachable from the spawn point, not under furniture, not on top of the
robot), with a seeded random generator, so a layout is reproducible run after run.

    ros2 launch vacuum_bringup vacuum.launch.py dirt:=15 dirt_seed:=3      # the normal way
    ros2 run vacuum_gazebo dirt_world.py --world W --map M --count 15 --seed 3 \\
        --preview dirt.png --closeups look.png                             # look first

What it writes (into --out-dir, default ~/vacuum_runs/dirt):
    <world>_dirt<N>_seed<S>.world   the original world + N dirt models (what Gazebo loads)
    <world>_dirt<N>_seed<S>.json    ground truth: every patch's name, kind, x, y, radius, scrub_m
    current_layout.json             a copy of the newest layout (what the dirt cleaner reads)
    <...>.png                       with --preview / --closeups: pictures of the layout

Dirt is four kinds of flat static models without collision, so it never touches the
robot's physics or the LiDAR plane (the Nav2 costmaps only use /scan):
    crumbs  tan / brown pieces of different size and shape, a few round ones, fine specks around
    dust    soft overlapping translucent grey discs
    spill   dark blob with satellite droplets and a glossy top (coffee, tea, cola)
    hair    two or three thin, slightly curved strands
A quarter of the patches sit in the band along walls and furniture, the places a lane
sweep tends to miss; the rest are spread over the open floor.

Every patch also has ``scrub_m``: how many metres the robot's brush must travel over it
before it comes off. Spills (and a share of the other patches, --stubborn-fraction) are
"stubborn" and need far more scrubbing than a crumb, so a single quick pass can leave
them behind: that is what a vision check after the main run has to find.

Map frame == world frame in this project (the robot spawns at the same coordinates in both).
"""

import argparse
import json
import math
import os
import random
import re
import shutil
import sys

import numpy as np

DIRT_HEIGHT = 0.004          # m, rough thickness of a patch (rug top is about 2.5 mm)
KINDS = ('crumbs', 'dust', 'spill', 'hair')
KIND_WEIGHTS = (0.40, 0.25, 0.20, 0.15)
SCRUB_NORMAL = 0.05          # m of brush travel over a patch to clear an ordinary one
SCRUB_STUBBORN = 0.50        # m for spills and the other stubborn patches

CRUMB_COLORS = [(0.62, 0.47, 0.28), (0.45, 0.30, 0.16), (0.80, 0.68, 0.45), (0.28, 0.19, 0.11)]
DUST_COLOR = (0.80, 0.79, 0.75)
SPILL_COLORS = [(0.14, 0.08, 0.04), (0.30, 0.13, 0.05), (0.09, 0.09, 0.11)]    # coffee, tea, cola
HAIR_COLORS = [(0.08, 0.06, 0.05), (0.35, 0.25, 0.15), (0.55, 0.52, 0.48)]
KIND_COLOR = {'crumbs': CRUMB_COLORS[0], 'dust': (0.55, 0.55, 0.55),
              'spill': SPILL_COLORS[0], 'hair': (0.0, 0.0, 0.0)}


# --------------------------------------------------------------------------
# Map reading (nav2_map_server format), row 0 = lowest y
# --------------------------------------------------------------------------

def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    pos, tokens = 0, []
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1                                    # single whitespace after maxval
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5' or maxval > 255:
        raise ValueError('%s: only 8-bit binary PGM (P5) maps are supported' % path)
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos).reshape(h, w)


class FloorMap:
    def __init__(self, free, blocked, resolution, origin_x, origin_y):
        self.free, self.blocked = free, blocked
        self.res, self.origin_x, self.origin_y = resolution, origin_x, origin_y

    @property
    def shape(self):
        return self.free.shape

    def world_to_cell(self, x, y):
        return (int(math.floor((y - self.origin_y) / self.res)),
                int(math.floor((x - self.origin_x) / self.res)))

    def cell_to_world(self, row, col):
        return self.origin_x + (col + 0.5) * self.res, self.origin_y + (row + 0.5) * self.res


def load_map(yaml_path):
    import yaml
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = meta['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img)
    px = read_pgm(img).astype(float)
    p = (px / 255.0) if int(meta.get('negate', 0)) else (255.0 - px) / 255.0   # occupancy probability
    p = p[::-1]                                                               # row 0 = lowest y
    free = p < float(meta.get('free_thresh', 0.196))
    blocked = ~free                                  # occupied and unknown both block
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return FloorMap(free, blocked, float(meta['resolution']), ox, oy)


# --------------------------------------------------------------------------
# Grid helpers
# --------------------------------------------------------------------------

def dilate(mask, r_cells):
    """Binary dilation by a disc (radius in cells, may be fractional)."""
    if r_cells <= 0:
        return mask.copy()
    n = int(math.floor(r_cells + 1e-9))
    h, w = mask.shape
    out = np.zeros_like(mask, dtype=bool)
    for dy in range(-n, n + 1):
        half = int(math.floor(math.sqrt(max(0.0, r_cells * r_cells - dy * dy)) + 1e-9))
        for dx in range(-half, half + 1):
            r0, r1 = max(0, dy), h + min(0, dy)
            c0, c1 = max(0, dx), w + min(0, dx)
            if r0 < r1 and c0 < c1:
                out[r0:r1, c0:c1] |= mask[r0 - dy:r1 - dy, c0 - dx:c1 - dx]
    return out


def component(mask, seed_rc):
    """4-connected region of ``mask`` that contains (or is nearest to) the seed cell."""
    h, w = mask.shape
    if not mask.any():
        raise ValueError('no free floor is wide enough for the robot in this map')
    r, c = seed_rc
    if not (0 <= r < h and 0 <= c < w and mask[r, c]):
        idx = np.argwhere(mask)
        near = idx[int(np.argmin((idx[:, 0] - r) ** 2 + (idx[:, 1] - c) ** 2))]
        r, c = int(near[0]), int(near[1])
    out = np.zeros_like(mask, dtype=bool)
    out[r, c] = True
    stack = [(r, c)]
    while stack:
        r, c = stack.pop()
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not out[rr, cc]:
                out[rr, cc] = True
                stack.append((rr, cc))
    return out


# --------------------------------------------------------------------------
# Where dirt may go
# --------------------------------------------------------------------------

def dirt_zones(fmap, spawn, min_clearance=0.10, robot_clearance=0.285, reach_radius=0.22,
               edge_band=0.35, spawn_clear=0.45):
    """(interior, edge) boolean masks of floor cells where a patch centre may be put.

    min_clearance    dirt keeps this far from walls/furniture (not squeezed under them)
    robot_clearance  how far from obstacles the robot's centre can get (radius + wall margin)
    reach_radius     floor within this of a reachable robot position counts as cleanable
    edge_band        cells closer than this to an obstacle are 'edge' cells
    spawn_clear      nothing within this of the spawn point (the robot sits there)
    """
    res = fmap.res
    reach = component(fmap.free & ~dilate(fmap.blocked, robot_clearance / res),
                      fmap.world_to_cell(*spawn))
    allowed = dilate(reach, reach_radius / res) & fmap.free & ~dilate(fmap.blocked, min_clearance / res)
    rows, cols = np.indices(fmap.shape)
    sx, sy = spawn
    wx = fmap.origin_x + (cols + 0.5) * res
    wy = fmap.origin_y + (rows + 0.5) * res
    allowed &= np.hypot(wx - sx, wy - sy) > spawn_clear
    edge = allowed & dilate(fmap.blocked, edge_band / res)
    return allowed & ~edge, edge


# --------------------------------------------------------------------------
# Patches: each is a handful of tiny visual shapes in the patch frame
# --------------------------------------------------------------------------
# shape dicts: shape 'box' (a, b, h) | 'cylinder' (r, h) | 'sphere' (r), plus dx, dy, yaw, color,
# optional specular (gloss, 0..1) and transparency (0 solid .. 1 invisible)

def _jitter(rng, color, amount):
    """Vary the brightness of a colour (same shift on every channel) so the hue stays natural."""
    d = rng.uniform(-amount, amount)
    return tuple(min(1.0, max(0.0, c + d + rng.uniform(-0.008, 0.008))) for c in color)


def _in_disc(rng, radius):
    rr, th = radius * math.sqrt(rng.random()), rng.uniform(0, 2 * math.pi)
    return rr * math.cos(th), rr * math.sin(th)


def _crumbs(rng, s):
    radius = rng.uniform(0.05, 0.10) * s
    base = rng.choice(CRUMB_COLORS)
    shapes = []
    for k in range(rng.randint(8, 18)):                 # pieces, irregular, different sizes
        dx, dy = _in_disc(rng, radius)
        col = _jitter(rng, base, 0.07)
        if rng.random() < 0.3:                          # a few round crumbs, half sunk into the floor
            r = rng.uniform(0.002, 0.005) * s
            shapes.append({'shape': 'sphere', 'dx': dx, 'dy': dy, 'r': r, 'color': col, 'specular': 0.1})
        else:
            a = rng.uniform(0.004, 0.013) * s
            shapes.append({'shape': 'box', 'dx': dx, 'dy': dy, 'a': a, 'b': a * rng.uniform(0.5, 1.4),
                           'yaw': rng.uniform(0, math.pi), 'h': rng.uniform(0.002, 0.006) + 0.0002 * k,
                           'color': col})
    for _ in range(rng.randint(6, 12)):                 # fine specks scattered wider
        dx, dy = _in_disc(rng, radius * 1.8)
        a = rng.uniform(0.002, 0.004) * s
        shapes.append({'shape': 'box', 'dx': dx, 'dy': dy, 'a': a, 'b': a, 'yaw': rng.uniform(0, math.pi),
                       'h': 0.002, 'color': _jitter(rng, base, 0.1), 'core': False})   # too small to label
    return shapes, radius * 1.4


def _dust(rng, s):
    radius = rng.uniform(0.06, 0.11) * s
    shapes = []
    n = rng.randint(5, 9)
    for k in range(n):                                   # big soft discs first, smaller ones on top
        dx, dy = _in_disc(rng, 0.5 * radius)
        r = radius * (0.55 - 0.40 * k / max(1, n - 1)) * rng.uniform(0.8, 1.2)
        shapes.append({'shape': 'cylinder', 'dx': dx, 'dy': dy, 'r': r, 'h': 0.0015 + 0.0003 * k,
                       'color': _jitter(rng, DUST_COLOR, 0.04), 'transparency': rng.uniform(0.35, 0.60)})
    for _ in range(rng.randint(5, 10)):                  # darker specks inside
        dx, dy = _in_disc(rng, radius * 0.8)
        a = rng.uniform(0.002, 0.005) * s
        shapes.append({'shape': 'box', 'dx': dx, 'dy': dy, 'a': a, 'b': a * rng.uniform(0.6, 1.2),
                       'yaw': rng.uniform(0, math.pi), 'h': 0.003, 'color': _jitter(rng, (0.45, 0.43, 0.40), 0.05)})
    return shapes, radius


def _spill(rng, s):
    radius = rng.uniform(0.04, 0.09) * s
    base = rng.choice(SPILL_COLORS)
    shapes = []
    lobes = rng.randint(4, 7)
    for k in range(lobes):                               # overlapping lobes make an irregular blob
        dx, dy = _in_disc(rng, 0.55 * radius)
        shapes.append({'shape': 'cylinder', 'dx': dx, 'dy': dy, 'r': radius * rng.uniform(0.30, 0.60),
                       'h': 0.0015 + 0.0002 * k, 'color': _jitter(rng, base, 0.02), 'specular': 0.15})
    for k in range(rng.randint(1, 2)):                   # glossy wet middle
        dx, dy = _in_disc(rng, 0.25 * radius)
        lighter = tuple(min(1.0, c + 0.08) for c in base)
        shapes.append({'shape': 'cylinder', 'dx': dx, 'dy': dy, 'r': radius * rng.uniform(0.15, 0.28),
                       'h': 0.0015 + 0.0002 * lobes + 0.0002 * k, 'color': lighter, 'specular': 0.8})
    for _ in range(rng.randint(2, 4)):                   # droplets around it
        ang, dist = rng.uniform(0, 2 * math.pi), radius * rng.uniform(1.1, 1.6)
        shapes.append({'shape': 'cylinder', 'dx': dist * math.cos(ang), 'dy': dist * math.sin(ang),
                       'r': rng.uniform(0.004, 0.012) * s, 'h': 0.0016, 'color': base, 'specular': 0.4})
    return shapes, radius * 1.6


def _hair(rng, s):
    shapes = []
    extent = 0.0
    for _ in range(rng.randint(2, 3)):
        x, y = _in_disc(rng, 0.04)
        heading = rng.uniform(0, 2 * math.pi)
        col = rng.choice(HAIR_COLORS)
        width = rng.uniform(0.002, 0.003) * s
        for k in range(rng.randint(3, 4)):               # a few straight pieces, each turned a little
            length = rng.uniform(0.02, 0.04) * s
            heading += rng.uniform(-0.7, 0.7)
            cx, cy = x + 0.5 * length * math.cos(heading), y + 0.5 * length * math.sin(heading)
            shapes.append({'shape': 'box', 'dx': cx, 'dy': cy, 'a': length, 'b': width, 'yaw': heading,
                           'h': 0.0015 + 0.0001 * k, 'color': _jitter(rng, col, 0.02)})
            x, y = x + length * math.cos(heading), y + length * math.sin(heading)
            extent = max(extent, math.hypot(x, y))
    return shapes, max(extent, 0.04)


_MAKERS = {'crumbs': _crumbs, 'dust': _dust, 'spill': _spill, 'hair': _hair}


def _convex_hull(points):
    """Andrew's monotone chain; points is a list of (x, y); returns the hull counter-clockwise."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def footprint_hull(shapes, core_only=True):
    """Outline of a patch on the floor, in the patch frame, as a list of [x, y] (metres).

    This is what the dataset labels are drawn around: the convex hull of every piece's
    footprint. ``core_only`` leaves out the tiny specks scattered far from the crumbs,
    which a camera cannot resolve and which would only blow the label box up.
    """
    pts = []
    for s in shapes:
        if core_only and s.get('core', True) is False:
            continue
        if s['shape'] == 'box':
            c, sn = math.cos(s.get('yaw', 0.0)), math.sin(s.get('yaw', 0.0))
            for u, v in ((-s['a'] / 2, -s['b'] / 2), (s['a'] / 2, -s['b'] / 2),
                         (s['a'] / 2, s['b'] / 2), (-s['a'] / 2, s['b'] / 2)):
                pts.append((s['dx'] + u * c - v * sn, s['dy'] + u * sn + v * c))
        else:
            for k in range(12):
                a = 2 * math.pi * k / 12
                pts.append((s['dx'] + s['r'] * math.cos(a), s['dy'] + s['r'] * math.sin(a)))
    return [[round(x, 4), round(y, 4)] for x, y in _convex_hull(pts)]


def patch_height(shapes, core_only=True):
    """Height of the tallest piece above the floor (metres)."""
    top = 0.0
    for s in shapes:
        if core_only and s.get('core', True) is False:
            continue
        top = max(top, 1.6 * s['r'] if s['shape'] == 'sphere' else s['h'])
    return round(top, 4)


def make_shapes(rng, kind, scale=1.0):
    """Visual shapes of one patch in the patch frame; returns (shapes, radius_m)."""
    return _MAKERS[kind](rng, scale)


def scatter_dirt(fmap, count, seed=1, spawn=(-2.0, -0.5), edge_fraction=0.25, min_spacing=0.45,
                 scale=1.0, stubborn_fraction=0.15, **zone_opts):
    """Pick ``count`` patches. Returns a list of dicts (name, kind, x, y, radius_m, scrub_m, shapes)."""
    if count < 1:
        raise ValueError('count must be at least 1')
    if scale <= 0:
        raise ValueError('scale must be positive')
    rng = random.Random(seed)
    interior, edge = dirt_zones(fmap, spawn, **zone_opts)
    cells = {'interior': np.argwhere(interior), 'edge': np.argwhere(edge)}
    if len(cells['interior']) + len(cells['edge']) == 0:
        raise ValueError('no floor left to put dirt on (map too small or too cluttered)')
    n_edge = int(round(count * edge_fraction))
    patches = []
    for i in range(count):
        zone = 'edge' if i < n_edge else 'interior'
        if len(cells[zone]) == 0:
            zone = 'interior' if zone == 'edge' else 'edge'
        placed = False
        for _ in range(300):
            row, col = cells[zone][rng.randrange(len(cells[zone]))]
            x, y = fmap.cell_to_world(int(row), int(col))
            x += rng.uniform(-0.5, 0.5) * fmap.res
            y += rng.uniform(-0.5, 0.5) * fmap.res
            if all(math.hypot(x - q['x'], y - q['y']) >= min_spacing for q in patches):
                placed = True
                break
        if not placed:
            sys.stderr.write('dirt: only %d of %d patches fit (map is crowded)\n' % (len(patches), count))
            break
        kind = rng.choices(KINDS, KIND_WEIGHTS)[0]
        shapes, radius = make_shapes(rng, kind, scale)
        stubborn = rng.random() < stubborn_fraction or kind == 'spill'     # always draw: seed-stable
        patches.append({'name': 'dirt_%03d' % len(patches), 'kind': kind, 'zone': zone,
                        'x': round(x, 3), 'y': round(y, 3), 'radius_m': round(radius, 3),
                        'stubborn': bool(stubborn),
                        'scrub_m': SCRUB_STUBBORN if stubborn else SCRUB_NORMAL,
                        'hull': footprint_hull(shapes), 'height_m': patch_height(shapes),
                        'shapes': shapes})
    return patches


# --------------------------------------------------------------------------
# SDF + world file
# --------------------------------------------------------------------------

def patch_sdf(p, indent='    '):
    lines = ["<model name='%s'>" % p['name'], '  <static>true</static>',
             '  <pose>%.4f %.4f 0 0 0 0</pose>' % (p['x'], p['y']), "  <link name='link'>"]
    for k, s in enumerate(p['shapes']):
        r, g, b = s['color']
        if s['shape'] == 'box':
            geom = '<box><size>%.4f %.4f %.4f</size></box>' % (s['a'], s['b'], s['h'])
            z = s['h'] / 2.0
        elif s['shape'] == 'sphere':
            geom = '<sphere><radius>%.4f</radius></sphere>' % s['r']
            z = s['r'] * 0.6                                   # lower part sinks into the floor
        else:
            geom = '<cylinder><radius>%.4f</radius><length>%.4f</length></cylinder>' % (s['r'], s['h'])
            z = s['h'] / 2.0
        sp = s.get('specular', 0.0)
        lines += ["    <visual name='v%d'>" % k,
                  '      <pose>%.4f %.4f %.5f 0 0 %.3f</pose>' % (s['dx'], s['dy'], z, s.get('yaw', 0.0)),
                  '      <cast_shadows>false</cast_shadows>']
        if s.get('transparency'):
            lines.append('      <transparency>%.2f</transparency>' % s['transparency'])
        lines += ['      <geometry>%s</geometry>' % geom,
                  '      <material><ambient>%.3f %.3f %.3f 1</ambient><diffuse>%.3f %.3f %.3f 1</diffuse>'
                  '<specular>%.2f %.2f %.2f 1</specular></material>' % (r, g, b, r, g, b, sp, sp, sp),
                  '    </visual>']
    lines += ['  </link>', '</model>']
    return ''.join(indent + line + '\n' for line in lines)


FLOOR_PALETTE = [(0.72, 0.58, 0.40), (0.30, 0.20, 0.13), (0.62, 0.62, 0.64), (0.88, 0.88, 0.86),
                 (0.76, 0.70, 0.58), (0.28, 0.28, 0.30), (0.62, 0.35, 0.25)]
# light oak, dark wood, grey tile, white tile, beige carpet, dark stone, terracotta


def _clamp(v):
    return min(1.0, max(0.0, v))


def environment_variation(seed):
    """Random floor colour and ambient light level for one episode (same seed, same result)."""
    rng = random.Random(seed * 7919 + 17)
    shade = rng.uniform(-0.06, 0.06)
    floor = tuple(round(_clamp(c + shade), 3) for c in rng.choice(FLOOR_PALETTE))
    level, warm = rng.uniform(0.25, 0.65), rng.uniform(-0.08, 0.08)
    ambient = (round(_clamp(level * (1 + warm)), 3), round(level, 3), round(_clamp(level * (1 - warm)), 3))
    return {'floor_color': list(floor), 'ambient': list(ambient)}


def set_scene_ambient(text, rgb):
    """Replace the <ambient> colour inside the world's <scene> (the overall light level)."""
    def rep(m):
        return '%s%.3f %.3f %.3f 1%s' % (m.group(1), rgb[0], rgb[1], rgb[2], m.group(2))
    return re.sub(r'(<scene>.*?<ambient>)[^<]*(</ambient>)', rep, text, count=1, flags=re.S)


def floor_slab_sdf(fmap, color, indent='    '):
    """A paper-thin coloured slab over the whole map, so the floor is not always the same grey."""
    h, w = fmap.shape
    sx, sy = w * fmap.res, h * fmap.res
    cx, cy = fmap.origin_x + sx / 2.0, fmap.origin_y + sy / 2.0
    r, g, b = color
    lines = ["<model name='floor_tint'>", '  <static>true</static>',
             '  <pose>%.4f %.4f 0.0003 0 0 0</pose>' % (cx, cy), "  <link name='link'>",
             "    <visual name='v'>", '      <cast_shadows>false</cast_shadows>',
             '      <geometry><box><size>%.3f %.3f 0.0006</size></box></geometry>' % (sx, sy),
             '      <material><ambient>%.3f %.3f %.3f 1</ambient><diffuse>%.3f %.3f %.3f 1</diffuse>'
             '<specular>0.05 0.05 0.05 1</specular></material>' % (r, g, b, r, g, b),
             '    </visual>', '  </link>', '</model>']
    return '    <!-- random floor colour (dirt_world.py, randomize option) -->\n' + \
        ''.join(indent + line + '\n' for line in lines)


LAYOUT_KEYS = ('name', 'kind', 'zone', 'x', 'y', 'radius_m', 'stubborn', 'scrub_m', 'height_m', 'hull')


def make_world(world_path, map_yaml, count, seed=1, spawn=(-2.0, -0.5),
               out_dir='~/vacuum_runs/dirt', randomize=False, **opts):
    """Write <world> with ``count`` dirt patches added; returns dict(world, layout, patches).

    randomize=True also gives the episode a random floor colour and light level (from the
    seed), so a vision model trained on many episodes does not learn one particular floor.
    """
    with open(world_path) as f:
        text = f.read()
    end = text.rfind('</world>')
    if end < 0:
        raise ValueError('%s has no </world>: not an SDF world file' % world_path)
    fmap = load_map(map_yaml)
    patches = scatter_dirt(fmap, count, seed, spawn, **opts)
    env, extra = None, ''
    if randomize:
        env = environment_variation(seed)
        text = set_scene_ambient(text, env['ambient'])
        end = text.rfind('</world>')
        extra = floor_slab_sdf(fmap, env['floor_color'])
    block = '\n    <!-- %d dirt patches, seed %d (dirt_world.py); visual only, no collision -->\n' % (
        len(patches), seed) + extra + ''.join(patch_sdf(p) for p in patches) + '  '
    out_dir = os.path.expanduser(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    stem = '%s_dirt%d_seed%d' % (os.path.splitext(os.path.basename(world_path))[0], len(patches), seed)
    out_world = os.path.join(out_dir, stem + '.world')
    with open(out_world, 'w') as f:
        f.write(text[:end] + block + text[end:])
    layout = os.path.join(out_dir, stem + '.json')
    with open(layout, 'w') as f:
        json.dump({'world': os.path.basename(world_path), 'map': os.path.abspath(map_yaml),
                   'seed': seed, 'count': len(patches), 'spawn': list(spawn),
                   'scale': opts.get('scale', 1.0), 'frame': 'world == map', 'environment': env,
                   'patches': [{k: p[k] for k in LAYOUT_KEYS} for p in patches]}, f, indent=2)
    current = os.path.join(out_dir, 'current_layout.json')      # what the dirt cleaner reads
    shutil.copyfile(layout, current)
    return {'world': out_world, 'layout': layout, 'current': current, 'patches': patches, 'stem': stem}


# --------------------------------------------------------------------------
# Pictures
# --------------------------------------------------------------------------

def _draw_patch(ax, p, ox=None, oy=None):
    """Draw one patch top-down at its own position (or at ox, oy) with matplotlib."""
    from matplotlib.patches import Circle, Polygon
    ox = p['x'] if ox is None else ox
    oy = p['y'] if oy is None else oy
    for s in p['shapes']:
        col = s['color']
        alpha = 1.0 - s.get('transparency', 0.0)
        cx, cy = ox + s['dx'], oy + s['dy']
        if s['shape'] == 'box':
            c, sn = math.cos(s.get('yaw', 0.0)), math.sin(s.get('yaw', 0.0))
            pts = [(cx + u * c - v * sn, cy + u * sn + v * c)
                   for u, v in ((-s['a'] / 2, -s['b'] / 2), (s['a'] / 2, -s['b'] / 2),
                                (s['a'] / 2, s['b'] / 2), (-s['a'] / 2, s['b'] / 2))]
            ax.add_patch(Polygon(pts, closed=True, fc=col, ec='none', alpha=alpha))
        else:
            ax.add_patch(Circle((cx, cy), s['r'], fc=col, ec='none', alpha=alpha))


def render_preview(png_path, map_yaml, patches, spawn):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fmap = load_map(map_yaml)
    h, w = fmap.shape
    extent = (fmap.origin_x, fmap.origin_x + w * fmap.res, fmap.origin_y, fmap.origin_y + h * fmap.res)
    img = np.full((h, w, 3), 0.45)                   # walls, furniture, unknown
    img[fmap.free] = (1.0, 1.0, 1.0)
    fig, ax = plt.subplots(figsize=(7, 7), dpi=110)
    ax.imshow(img, origin='lower', extent=extent, interpolation='nearest')
    for p in patches:
        _draw_patch(ax, p)
        ax.add_patch(Circle((p['x'], p['y']), max(p['radius_m'], 0.05), fill=False,
                            ec='#d9480f' if p.get('stubborn') else '#868e96', lw=0.8))
    ax.plot(*spawn, 'o', color='#2b8a3e', ms=7)
    handles = [plt.Line2D([], [], marker='o', ls='', color='none', mec='#868e96', label='ordinary'),
               plt.Line2D([], [], marker='o', ls='', color='none', mec='#d9480f', label='stubborn'),
               plt.Line2D([], [], marker='o', ls='', color='#2b8a3e', label='spawn')]
    ax.legend(handles=handles, loc='upper right', fontsize=8)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('%d dirt patches (rings: ordinary / stubborn)' % len(patches), fontsize=10)
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)


def render_closeups(png_path, patches, per_kind=4):
    """Top-down close-ups of several patches of every kind on two floor colours."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    floors = [(0.93, 0.90, 0.84), (0.55, 0.45, 0.35)]      # light rug, wooden floor
    chosen = []
    for kind in KINDS:
        chosen += [p for p in patches if p['kind'] == kind][:per_kind]
    cols = per_kind
    rows = int(math.ceil(len(chosen) / cols)) or 1
    fig, axes = plt.subplots(rows, cols, figsize=(2.4 * cols, 2.4 * rows), dpi=110, squeeze=False)
    for ax in axes.flat:
        ax.axis('off')
    for i, p in enumerate(chosen):
        ax = axes.flat[i]
        half = max(0.13, p['radius_m'] * 1.25)
        ax.set_facecolor(floors[i % 2])
        ax.add_patch(plt.Rectangle((-half, -half), 2 * half, 2 * half, fc=floors[i % 2], ec='none'))
        _draw_patch(ax, p, 0.0, 0.0)
        ax.set_xlim(-half, half)
        ax.set_ylim(-half, half)
        ax.set_aspect('equal')
        ax.set_title('%s%s  r=%.0f cm' % (p['kind'], ' (stubborn)' if p.get('stubborn') else '',
                                          100 * p['radius_m']), fontsize=8)
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', required=True, help='SDF world file')
    ap.add_argument('--map', required=True, help='map yaml of that world (nav2_map_server format)')
    ap.add_argument('--count', type=int, default=15)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--spawn', nargs=2, type=float, default=[-2.0, -0.5], metavar=('X', 'Y'))
    ap.add_argument('--out-dir', default='~/vacuum_runs/dirt')
    ap.add_argument('--edge-fraction', type=float, default=0.25,
                    help='share of patches placed in the band along walls and furniture')
    ap.add_argument('--scale', type=float, default=1.0, help='size multiplier for every patch')
    ap.add_argument('--stubborn-fraction', type=float, default=0.15,
                    help='share of non-spill patches that need much more scrubbing')
    ap.add_argument('--randomize', action='store_true',
                    help='random floor colour and light level for this seed (for dataset episodes)')
    ap.add_argument('--preview', metavar='PNG', help='also draw the layout on the map')
    ap.add_argument('--closeups', metavar='PNG', help='also draw close-ups of the patches')
    args = ap.parse_args(argv)
    res = make_world(args.world, args.map, args.count, args.seed, tuple(args.spawn), args.out_dir,
                     randomize=args.randomize, edge_fraction=args.edge_fraction, scale=args.scale,
                     stubborn_fraction=args.stubborn_fraction)
    print('world :', res['world'])
    print('layout:', res['layout'])
    if args.preview:
        render_preview(args.preview, args.map, res['patches'], tuple(args.spawn))
        print('preview:', args.preview)
    if args.closeups:
        render_closeups(args.closeups, res['patches'])
        print('closeups:', args.closeups)


if __name__ == '__main__':
    main()
