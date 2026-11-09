"""Coverage planning core: wall pass + boustrophedon lanes + coverage metric.

Pure numpy, no ROS imports, so it can be unit-tested and previewed offline.

Everything works on a 2-D occupancy grid in nav_msgs/OccupancyGrid layout:
``data[row, col]``, row 0 at the map origin (lowest y), values 0 = free,
100 = occupied, -1 = unknown. The map origin yaw is assumed to be 0 (true for
maps written by nav2_map_server / slam_toolbox).

Idea
----
1. Robot *centres* may only be where the whole robot fits, i.e. at least
   ``robot_radius + wall_margin`` away from any occupied/unknown cell. That set,
   restricted to the connected piece the robot is in, is ``reach``.
2. Wall pass: the boundary of ``reach`` is a set of closed loops, one along the
   room walls and one around each piece of furniture. Following them cleans the
   edges.
3. Lanes: parallel straight lines through ``reach``, ``lane_spacing`` apart, cut
   into segments wherever they leave ``reach``; visited greedily nearest-first.
4. Metric: floor cells that some reachable robot centre could clean (within
   ``clean_radius``) are the 100 %; cells a robot trail has passed within
   ``clean_radius`` of are "covered".
"""

from collections import deque
from dataclasses import dataclass, field
import math
import os

import numpy as np


# --------------------------------------------------------------------------
# Map container and loading
# --------------------------------------------------------------------------

@dataclass
class GridMap:
    data: np.ndarray          # (h, w) int8, row 0 = lowest y
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def height(self):
        return self.data.shape[0]

    @property
    def width(self):
        return self.data.shape[1]

    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def cell_to_world(self, row, col):
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    @classmethod
    def from_occupancy_grid(cls, width, height, resolution, origin_x, origin_y,
                            flat_data):
        data = np.asarray(flat_data, dtype=np.int8).reshape(height, width)
        return cls(data, float(resolution), float(origin_x), float(origin_y))


def read_pgm(path):
    """Read a binary (P5) or ASCII (P2) PGM into a (h, w) uint8 array, top row first."""
    with open(path, 'rb') as f:
        raw = f.read()
    tokens = []
    i = 0
    n = len(raw)
    # header: magic, width, height, maxval (comments start with '#')
    while len(tokens) < 4:
        while i < n and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while i < n and raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < n and not raw[j:j + 1].isspace():
            j += 1
        tokens.append(raw[i:j])
        i = j
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError('16-bit PGM not supported')
    if magic == b'P5':
        i += 1  # single whitespace after maxval
        pix = np.frombuffer(raw, dtype=np.uint8, count=w * h, offset=i)
    elif magic == b'P2':
        pix = np.array(raw[i:].split(), dtype=np.uint8)[:w * h]
    else:
        raise ValueError('unsupported PGM type %r' % magic)
    return pix.reshape(h, w).copy()


def load_map_yaml(path):
    """Load a nav2_map_server style map (yaml + pgm) into a GridMap."""
    import yaml
    with open(path) as f:
        meta = yaml.safe_load(f)
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(path)), image)
    pix = read_pgm(image).astype(np.float64)
    negate = int(meta.get('negate', 0))
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.196))
    p = pix / 255.0 if negate else (255.0 - pix) / 255.0
    data = np.full(pix.shape, -1, dtype=np.int8)
    data[p > occ_t] = 100
    data[p < free_t] = 0
    data = data[::-1].copy()            # image top row = max y
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return GridMap(data, float(meta['resolution']), ox, oy)


# --------------------------------------------------------------------------
# Small grid helpers
# --------------------------------------------------------------------------

def disc_row_halfwidths(r_cells):
    """For dy in -floor(r)..floor(r) the half width of a disc of radius r (cells, may be fractional)."""
    n = int(math.floor(r_cells + 1e-9))
    return [int(math.floor(math.sqrt(max(0.0, r_cells * r_cells - dy * dy)) + 1e-9))
            for dy in range(-n, n + 1)]


def dilate(mask, r_cells):
    """Binary dilation by a disc of radius r_cells (cells, may be fractional)."""
    if r_cells <= 0:
        return mask.copy()
    n = int(math.floor(r_cells + 1e-9))
    h, w = mask.shape
    out = np.zeros_like(mask, dtype=bool)
    hw = disc_row_halfwidths(r_cells)
    for dy, half in zip(range(-n, n + 1), hw):
        for dx in range(-half, half + 1):
            # out[r, c] |= mask[r - dy, c - dx]
            r0, r1 = max(0, dy), h + min(0, dy)
            c0, c1 = max(0, dx), w + min(0, dx)
            if r0 < r1 and c0 < c1:
                out[r0:r1, c0:c1] |= mask[r0 - dy:r1 - dy, c0 - dx:c1 - dx]
    return out


def flood_fill(mask, seed):
    """4-connected component of ``mask`` containing ``seed`` (row, col)."""
    h, w = mask.shape
    out = np.zeros((h, w), dtype=bool)
    if not (0 <= seed[0] < h and 0 <= seed[1] < w) or not mask[seed]:
        return out
    out[seed] = True
    dq = deque([seed])
    while dq:
        r, c = dq.popleft()
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not out[rr, cc]:
                out[rr, cc] = True
                dq.append((rr, cc))
    return out


def label_components(mask):
    """Return a list of boolean masks, one per 4-connected component."""
    remaining = mask.copy()
    comps = []
    while True:
        idx = np.argwhere(remaining)
        if len(idx) == 0:
            break
        comp = flood_fill(remaining, (int(idx[0][0]), int(idx[0][1])))
        comps.append(comp)
        remaining &= ~comp
    return comps


# --------------------------------------------------------------------------
# Parameters and masks
# --------------------------------------------------------------------------

@dataclass
class CoverageParams:
    robot_radius: float = 0.1745   # m, inscribed radius of the robot body
    wall_margin: float = 0.06      # m, extra clearance kept from walls/furniture
    clean_radius: float = 0.22     # m, floor within this of the robot centre counts as cleaned (body + side brush)
    lane_spacing: float = 0.30     # m between lanes (~body width with some overlap)
    lane_axis: str = 'auto'        # 'x', 'y' or 'auto' (pick the one with fewer segments)
    path_step: float = 0.08        # m between poses in the generated paths
    min_segment: float = 0.25      # m, drop lane segments shorter than this
    min_loop: float = 0.8          # m, drop wall loops shorter than this
    merge_max: float = 1.5         # m, join lane segments into one path if a clear straight link is this short
    smooth_window: float = 0.25    # m, moving-average window for wall loops
    wall_pass: bool = True
    lanes: bool = True


@dataclass
class Masks:
    blocked: np.ndarray       # occupied or unknown
    forbidden: np.ndarray     # robot centre may not be here
    reach: np.ndarray         # centres the robot can reach (one connected piece)
    coverable: np.ndarray     # free floor some reachable centre can clean
    start_cell: tuple
    warnings: list = field(default_factory=list)


def clearance_cells(params, res):
    """Centre-to-centre clearance from blocked cells, in (fractional) cells."""
    return (params.robot_radius + params.wall_margin) / res


def build_masks(grid, params, start_xy):
    res = grid.resolution
    blocked = grid.data != 0
    forbidden = dilate(blocked, clearance_cells(params, res))
    allowed = ~forbidden
    warnings = []
    if not allowed.any():
        raise ValueError('no free space is wide enough for the robot '
                         '(robot_radius + wall_margin too large for this map?)')

    srow, scol = grid.world_to_cell(*start_xy)
    comps = label_components(allowed)
    comps.sort(key=lambda m: -int(m.sum()))
    seed_comp = None
    if grid.in_bounds(srow, scol):
        for comp in comps:
            if comp[srow, scol]:
                seed_comp = comp
                break
    if seed_comp is None:
        idx = np.argwhere(allowed)
        d2 = (idx[:, 0] - srow) ** 2 + (idx[:, 1] - scol) ** 2
        nearest = idx[int(np.argmin(d2))]
        for comp in comps:
            if comp[nearest[0], nearest[1]]:
                seed_comp = comp
                break
        warnings.append('robot start is not on a reachable cell; used the nearest one')
    if seed_comp.sum() < 0.5 * comps[0].sum():
        warnings.append('robot is in a small pocket; planning the largest area instead')
        seed_comp = comps[0]
    reach = seed_comp
    if grid.in_bounds(srow, scol) and reach[srow, scol]:
        start_cell = (srow, scol)
    else:
        idx = np.argwhere(reach)
        d2 = (idx[:, 0] - srow) ** 2 + (idx[:, 1] - scol) ** 2
        start_cell = tuple(int(v) for v in idx[int(np.argmin(d2))])

    coverable = dilate(reach, params.clean_radius / res) & (grid.data == 0)
    return Masks(blocked, forbidden, reach, coverable, start_cell, warnings)


# --------------------------------------------------------------------------
# Wall pass: closed loops along the boundary of `reach`
# --------------------------------------------------------------------------

_DIRS = ((1, 0), (0, 1), (-1, 0), (0, -1))   # +x, +y, -x, -y


def boundary_loops(region):
    """Trace every boundary of a boolean region as a closed loop of cells.

    Each unit edge between a region cell and a non-region cell is directed so
    that the region is on its left; edges then chain into closed loops (outer
    boundary counter-clockwise, holes clockwise). Returns a list of lists of
    (row, col) cells (the region cell on the inside of each edge, consecutive
    duplicates removed).
    """
    pad = np.pad(region, 1, constant_values=False)
    edges = []              # (v0, v1, dir, cell) with vertices as (x=col, y=row)
    out_edges = {}
    rows, cols = np.nonzero(region)
    for r, c in zip(rows.tolist(), cols.tolist()):
        pr, pc = r + 1, c + 1
        cand = []
        if not pad[pr - 1, pc]:
            cand.append(((c, r), (c + 1, r), 0))
        if not pad[pr, pc + 1]:
            cand.append(((c + 1, r), (c + 1, r + 1), 1))
        if not pad[pr + 1, pc]:
            cand.append(((c + 1, r + 1), (c, r + 1), 2))
        if not pad[pr, pc - 1]:
            cand.append(((c, r + 1), (c, r), 3))
        for v0, v1, d in cand:
            out_edges.setdefault(v0, []).append(len(edges))
            edges.append((v0, v1, d, (r, c)))

    used = [False] * len(edges)
    loops = []
    for start in range(len(edges)):
        if used[start]:
            continue
        cells = []
        cur = start
        while True:
            used[cur] = True
            v0, v1, d, cell = edges[cur]
            if not cells or cells[-1] != cell:
                cells.append(cell)
            nxt = None
            best_pref = None
            for cand in out_edges.get(v1, []):
                if used[cand]:
                    continue
                turn = (edges[cand][2] - d) % 4
                pref = {1: 0, 0: 1, 3: 2, 2: 3}[turn]   # prefer left, straight, right
                if best_pref is None or pref < best_pref:
                    best_pref, nxt = pref, cand
            if nxt is None:
                break
            cur = nxt
        if len(cells) > 1 and cells[0] == cells[-1]:
            cells.pop()
        loops.append(cells)
    return loops


def _polyline_length(pts, closed=False):
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return 0.0
    seg = np.diff(pts, axis=0)
    total = float(np.hypot(seg[:, 0], seg[:, 1]).sum())
    if closed:
        total += float(np.hypot(*(pts[0] - pts[-1])))
    return total


def _signed_area(pts):
    pts = np.asarray(pts, dtype=float)
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def resample_closed(pts, step):
    """Resample a closed polyline at uniform arc-length spacing (first point not repeated)."""
    pts = np.asarray(pts, dtype=float)
    ext = np.vstack([pts, pts[:1]])
    seg = np.hypot(*np.diff(ext, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = max(8, int(round(total / step)))
    targets = np.linspace(0.0, total, n, endpoint=False)
    return np.column_stack([np.interp(targets, s, ext[:, 0]),
                            np.interp(targets, s, ext[:, 1])])


def smooth_closed(pts, k):
    """Circular moving average with half-window k (points)."""
    if k <= 0 or len(pts) < 2 * k + 3:
        return np.asarray(pts, dtype=float)
    pts = np.asarray(pts, dtype=float)
    kernel = np.ones(2 * k + 1) / (2 * k + 1)
    xs = np.convolve(np.pad(pts[:, 0], (k, k), mode='wrap'), kernel, mode='valid')
    ys = np.convolve(np.pad(pts[:, 1], (k, k), mode='wrap'), kernel, mode='valid')
    return np.column_stack([xs, ys])


def wall_loops(grid, masks, params):
    """World-space wall-pass loops (list of (N, 2) arrays, closed, first point not repeated).

    Outer boundary first, then furniture loops.
    """
    res = grid.resolution
    loops = []
    for cells in boundary_loops(masks.reach):
        if len(cells) < 4:
            continue
        pts = np.array([grid.cell_to_world(r, c) for r, c in cells])
        if _polyline_length(pts, closed=True) < params.min_loop:
            continue
        base = resample_closed(pts, res)
        k = int(round(params.smooth_window / res / 2.0))
        sm = smooth_closed(base, k)
        # never trust smoothing into a cell the robot cannot be in
        def _ok(p):
            r, c = grid.world_to_cell(p[0], p[1])
            return grid.in_bounds(r, c) and masks.reach[r, c]

        for i in range(len(sm)):
            if _ok(sm[i]):
                continue
            sm[i] = base[i]
            if not _ok(sm[i]):
                # interpolating across a diagonal step can clip a corner cell:
                # snap to the nearest boundary cell centre, which is always reachable
                sm[i] = pts[int(np.argmin(np.hypot(pts[:, 0] - base[i, 0], pts[:, 1] - base[i, 1])))]
        loops.append(sm)
    loops.sort(key=lambda p: -abs(_signed_area(p)))
    return loops


# --------------------------------------------------------------------------
# Lanes
# --------------------------------------------------------------------------

def _runs(row_mask):
    """(start, end) index pairs (inclusive) of consecutive True values."""
    padded = np.concatenate([[0], row_mask.astype(np.int8), [0]])
    d = np.diff(padded)
    starts = np.nonzero(d == 1)[0]
    ends = np.nonzero(d == -1)[0] - 1
    return list(zip(starts.tolist(), ends.tolist()))


def lane_segments(grid, masks, params, axis):
    """Straight lane segments as ((x0, y0), (x1, y1)) world points.

    axis 'x': lanes run along x (constant y). axis 'y': along y.
    """
    res = grid.resolution
    mask = masks.reach if axis == 'x' else masks.reach.T
    rows = np.nonzero(mask.any(axis=1))[0]
    if len(rows) == 0:
        return []
    rmin, rmax = int(rows.min()), int(rows.max())
    span = (rmax - rmin) * res
    s = params.lane_spacing
    if span <= 2 * s:
        lane_rows = [int(round((rmin + rmax) / 2.0))]
    else:
        n = int(math.ceil((span - 2 * s) / s)) + 1
        lane_rows = sorted(set(int(round(v)) for v in
                               np.linspace(rmin + s / res, rmax - s / res, n)))
    segs = []
    for r in lane_rows:
        for c0, c1 in _runs(mask[r]):
            if (c1 - c0) * res < params.min_segment:
                continue
            if axis == 'x':
                a = grid.cell_to_world(r, c0)
                b = grid.cell_to_world(r, c1)
            else:
                a = grid.cell_to_world(c0, r)
                b = grid.cell_to_world(c1, r)
            segs.append((a, b))
    return segs


def _line_clear(grid, reach, a, b):
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(d / (grid.resolution * 0.5)) + 1)
    for t in np.linspace(0.0, 1.0, n):
        r, c = grid.world_to_cell(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if not (grid.in_bounds(r, c) and reach[r, c]):
            return False
    return True


def order_segments(grid, masks, params, segments, start_xy):
    """Greedy nearest-endpoint ordering; returns a list of corner-point lists.

    Consecutive segments whose joining straight line is clear (and short) are
    merged into one piece, so the robot drives a continuous zig-zag.
    """
    remaining = list(segments)
    cur = tuple(start_xy)
    pieces = []
    piece = None
    while remaining:
        best = None
        for i, (a, b) in enumerate(remaining):
            for flip, (p, _q) in ((False, (a, b)), (True, (b, a))):
                d = math.hypot(p[0] - cur[0], p[1] - cur[1])
                if best is None or d < best[0]:
                    best = (d, i, flip)
        d, i, flip = best
        a, b = remaining.pop(i)
        if flip:
            a, b = b, a
        if piece is not None and d <= params.merge_max and _line_clear(grid, masks.reach, cur, a):
            piece.extend([a, b])
        else:
            if piece is not None:
                pieces.append(piece)
            piece = [a, b]
        cur = b
    if piece is not None:
        pieces.append(piece)
    return pieces


def densify_open(corners, step):
    """Interpolate a polyline through its corners at ~step spacing, keeping every corner."""
    out = [tuple(corners[0])]
    for p, q in zip(corners[:-1], corners[1:]):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(1, int(math.ceil(d / step)))
        for k in range(1, n + 1):
            t = k / n
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def with_yaw(points):
    """[(x, y)] -> [(x, y, yaw)] with yaw along the path (last pose keeps the previous yaw)."""
    out = []
    n = len(points)
    yaw = 0.0
    for i in range(n):
        if i < n - 1:
            dx = points[i + 1][0] - points[i][0]
            dy = points[i + 1][1] - points[i][1]
            if math.hypot(dx, dy) > 1e-9:
                yaw = math.atan2(dy, dx)
        out.append((points[i][0], points[i][1], yaw))
    return out


# --------------------------------------------------------------------------
# Plan
# --------------------------------------------------------------------------

@dataclass
class Piece:
    kind: str                  # 'wall' or 'lane'
    poses: list                # [(x, y, yaw)]

    @property
    def length(self):
        return _polyline_length([(p[0], p[1]) for p in self.poses])


@dataclass
class Plan:
    pieces: list
    masks: Masks
    params: CoverageParams
    lane_axis: str = ''

    @property
    def warnings(self):
        return self.masks.warnings

    def stats(self):
        wall = [p for p in self.pieces if p.kind == 'wall']
        lane = [p for p in self.pieces if p.kind == 'lane']
        return {
            'pieces': len(self.pieces),
            'wall_loops': len(wall),
            'lane_pieces': len(lane),
            'wall_length_m': round(sum(p.length for p in wall), 2),
            'lane_length_m': round(sum(p.length for p in lane), 2),
            'lane_axis': self.lane_axis,
        }


def plan_coverage(grid, params, start_xy):
    masks = build_masks(grid, params, start_xy)
    pieces = []
    cur = tuple(start_xy)
    axis_used = ''

    if params.wall_pass:
        loops = wall_loops(grid, masks, params)
        todo = list(loops)
        # outer boundary (largest) first, then the others nearest-first
        ordered = []
        if todo:
            ordered.append(todo.pop(0))
        while todo:
            ref = ordered[-1][0]
            todo.sort(key=lambda p: float(np.min(np.hypot(p[:, 0] - ref[0], p[:, 1] - ref[1]))))
            ordered.append(todo.pop(0))
        for loop in ordered:
            d = np.hypot(loop[:, 0] - cur[0], loop[:, 1] - cur[1])
            loop = np.roll(loop, -int(np.argmin(d)), axis=0)
            pts = resample_closed(loop, params.path_step)
            pts = np.vstack([pts, pts[:1]])           # close the loop
            poses = with_yaw([tuple(p) for p in pts])
            pieces.append(Piece('wall', poses))
            cur = (pts[-1][0], pts[-1][1])

    if params.lanes:
        axes = ['x', 'y'] if params.lane_axis == 'auto' else [params.lane_axis]
        cands = {ax: lane_segments(grid, masks, params, ax) for ax in axes}
        axis_used = min(axes, key=lambda ax: len(cands[ax]))
        segs = cands[axis_used]
        for corners in order_segments(grid, masks, params, segs, cur):
            pts = densify_open(corners, params.path_step)
            pieces.append(Piece('lane', with_yaw(pts)))

    return Plan(pieces, masks, params, axis_used)


# --------------------------------------------------------------------------
# Coverage metric
# --------------------------------------------------------------------------

class CoverageTracker:
    """Marks floor cells within ``clean_radius`` of visited robot positions."""

    def __init__(self, grid, coverable, clean_radius):
        self.grid = grid
        self.coverable = coverable
        self.r = clean_radius / grid.resolution
        self.n = int(math.floor(self.r + 1e-9))
        self.hw = disc_row_halfwidths(self.r)
        self.covered = np.zeros(coverable.shape, dtype=bool)
        self.total = int(coverable.sum())
        self.dirty = False

    def stamp_cell(self, row, col):
        h, w = self.covered.shape
        for dy, half in zip(range(-self.n, self.n + 1), self.hw):
            rr = row + dy
            if 0 <= rr < h:
                c0, c1 = max(0, col - half), min(w, col + half + 1)
                if c0 < c1:
                    self.covered[rr, c0:c1] = True
        self.dirty = True

    def stamp_xy(self, x, y):
        row, col = self.grid.world_to_cell(x, y)
        if self.grid.in_bounds(row, col):
            self.stamp_cell(row, col)

    def percent(self):
        if self.total == 0:
            return 0.0
        return 100.0 * float((self.covered & self.coverable).sum()) / self.total
