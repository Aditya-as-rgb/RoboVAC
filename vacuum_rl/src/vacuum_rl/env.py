"""Gymnasium re-clean stage following a deterministic offline coverage sweep."""

from dataclasses import dataclass, replace
from functools import lru_cache
import math
from pathlib import Path

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from .navigation import GridRoutes
from .shared import core, dirt_cleaner, dirt_world, reclean, sim, validate

DEFAULT_MAP = Path(__file__).parent / "data" / "living_room.yaml"
TRAIN_DIRT_SEEDS = 512
KINDS = dirt_world.KINDS


@dataclass(frozen=True)
class DetectorConfig:
    """Rates in crumbs, dust, spill, hair order."""

    recall: tuple = (.77, .29, .83, .60)
    precision: tuple = (.80, .60, .85, .70)
    position_std: float = .075
    rate_jitter: float = .12

    def __post_init__(self):
        for rates in (self.recall, self.precision):
            if len(rates) != len(KINDS) or not all(0 <= rate <= 1 for rate in rates):
                raise ValueError("Provide four rates in [0, 1] in dirt class order")
        if min(self.precision) <= 0 or not (0 <= self.position_std < math.inf
                                           and 0 <= self.rate_jitter <= 1):
            raise ValueError("Precision must be positive and detector noise finite and nonnegative")

    @classmethod
    def perfect(cls):
        return cls((1.,) * 4, (1.,) * 4, 0., 0.)


@dataclass
class Candidate:
    node: int
    xy: tuple
    confidence: float
    kind: int
    visits: int = 0


@lru_cache(maxsize=4)
def _room(map_path, spawn):
    grid = core.load_map_yaml(map_path)
    params = core.CoverageParams()
    plan, _ = validate.plan_validated(grid, params, spawn)
    nav_masks = core.build_masks(grid, replace(params, wall_margin=.02), spawn)
    routes = GridRoutes(grid, nav_masks.reach)
    trails, _, _ = sim.run_plan(plan, (*spawn, 0.))
    points = [tuple(routes.xy[routes.nearest(spawn)])]
    for trail in trails:
        start = routes.nearest(trail[0])
        transit = routes.path(routes.nearest(points[-1]), start)
        points.extend(map(tuple, routes.xy[transit]))
        points.extend(trail)
    finish = routes.nearest(points[-1])
    points.append(tuple(routes.xy[finish]))
    # Feed the brush the same minimum pose separation it uses in the ROS run.
    samples = [points[0]]
    for point in points[1:]:
        if math.dist(point, samples[-1]) >= .02:
            samples.append(point)
    tracker = core.CoverageTracker(grid, plan.masks.coverable, plan.params.clean_radius)
    for x, y in samples:
        tracker.stamp_xy(x, y)
    return grid, plan, routes, tuple(samples), tracker.covered, finish, dirt_world.load_map(map_path)


@lru_cache(maxsize=1024)
def _dirt_after_sweep(map_path, spawn, count, brush_radius, seed):
    _, _, _, sweep, _, _, floor = _room(map_path, spawn)
    layout = dirt_world.scatter_dirt(floor, count, seed, spawn)
    patches = [dirt_cleaner.Patch(**{k: p[k] for k in (
        "name", "kind", "x", "y", "radius_m", "stubborn", "scrub_m", "zone")}) for p in layout]
    field = dirt_cleaner.DirtField(patches, brush_radius)
    elapsed = 0.
    previous = sweep[0]
    for point in sweep:
        elapsed += math.dist(previous, point) / .25
        field.update(*point, elapsed)
        previous = point
    return tuple(patches)


class RecleanEnv(gym.Env):
    """One episode starts where the existing coverage planner ends its first sweep."""

    metadata = {"render_modes": []}

    def __init__(self, map_path=DEFAULT_MAP, spawn=(-2., -.5), dirt_count=25, budget_s=75.,
                 max_candidates=24, max_visits=30, brush_radius=.20, speed=.25,
                 detector=None, training=False):
        super().__init__()
        if (min(dirt_count, max_candidates, max_visits) < 1
                or not all(math.isfinite(v) and v > 0 for v in (budget_s, brush_radius, speed))):
            raise ValueError("Counts, budget, brush radius, and speed must be positive")
        self.map_path = str(Path(map_path).expanduser().resolve())
        self.spawn = tuple(spawn)
        self.dirt_count = dirt_count
        self.budget_s, self.max_candidates, self.max_visits = budget_s, max_candidates, max_visits
        self.brush_radius, self.speed = brush_radius, speed
        self.detector = detector or DetectorConfig()
        self.training = training
        self.grid, self.plan, self.routes, _, self.sweep_covered, self.start_node, _ = _room(
            self.map_path, self.spawn)
        self.origin = np.array([self.grid.origin_x, self.grid.origin_y])
        self.span = np.array([self.grid.width, self.grid.height]) * self.grid.resolution
        self.stop_action = max_candidates
        self.action_space = spaces.Discrete(max_candidates + 1)
        # Spot features: normalized x, y, confidence, four-class one-hot, visit count.
        low = np.zeros((max_candidates, 8), dtype=np.float32)
        high = np.ones_like(low)
        high[:, -1] = max_visits
        self.observation_space = spaces.Dict({
            "position": spaces.Box(0., 1., shape=(2,), dtype=np.float32),
            "time_remaining": spaces.Box(0., budget_s, shape=(1,), dtype=np.float32),
            "candidates": spaces.Box(low, high, dtype=np.float32),
            "mask": spaces.MultiBinary(max_candidates),
        })

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        dirt_seed = options.get("dirt_seed")
        if dirt_seed is None:
            dirt_seed = (seed if seed is not None and not self.training else
                         int(self.np_random.integers(TRAIN_DIRT_SEEDS)))
        initial = _dirt_after_sweep(self.map_path, self.spawn, self.dirt_count,
                                    self.brush_radius, int(dirt_seed))
        self.field = dirt_cleaner.DirtField([replace(p) for p in initial], self.brush_radius)
        self.node = self.start_node
        self.field.update(*self.routes.xy[self.node], 0.)
        self.elapsed = self.travelled = 0.
        self.false_visits = self.visits = 0
        self.episode_over = False
        self.dirt_seed = int(dirt_seed)
        self.remaining_at_start = self.field.remaining
        self.candidates = self._detect(self.detector)
        self._order_candidates()
        return self._observe(), self._info()

    def _detect(self, config):
        rng = self.np_random
        recall = np.asarray(config.recall, dtype=float)
        precision = np.asarray(config.precision, dtype=float)
        if self.training and config.rate_jitter:
            recall = np.clip(recall + rng.uniform(-config.rate_jitter, config.rate_jitter, 4), 0., 1.)
            precision = np.clip(precision + rng.uniform(-config.rate_jitter, config.rate_jitter, 4),
                                .05, 1.)
        perfect = np.all(recall == 1.) and np.all(precision == 1.) and config.position_std == 0.
        live = [p for p in self.field.patches if p.removed_at is None]
        candidates = []
        for p in live:
            kind = KINDS.index(p.kind)
            if rng.random() < recall[kind]:
                xy = np.clip(np.array([p.x, p.y]) + rng.normal(0, config.position_std, 2),
                             self.origin, self.origin + self.span)
                confidence = 1. if perfect else float(rng.beta(7, 3))
                candidates.append(Candidate(self.routes.nearest(xy), tuple(xy), confidence, kind))
        counts = np.bincount([KINDS.index(p.kind) for p in live], minlength=len(KINDS))
        false_counts = rng.poisson(counts * recall * (1. - precision) / precision)
        if false_counts.any():
            positions = np.array([(p.x, p.y) for p in live])
            squares = np.square(self.routes.xy[:, None, :] - positions).sum(axis=2)
            cells = np.flatnonzero((squares > self.brush_radius ** 2).all(axis=1))
            for kind, count in enumerate(false_counts):
                for node in rng.choice(cells, size=min(count, len(cells), self.max_candidates),
                                       replace=False):
                    candidates.append(Candidate(int(node), tuple(self.routes.xy[node]),
                                                float(rng.beta(4, 5)), kind))
        return sorted(candidates, key=lambda c: -c.confidence)[:self.max_candidates]

    def _order_candidates(self):
        self.candidates.sort(key=lambda candidate: self.routes.distance(self.node, candidate.node))

    def _time_to(self, node):
        return self.routes.visit_distance(self.node, node) / self.speed + .5

    def action_masks(self):
        mask = np.zeros(self.action_space.n, dtype=bool)
        if not self.episode_over:
            for i, candidate in enumerate(self.candidates):
                mask[i] = self.elapsed + self._time_to(candidate.node) <= self.budget_s + 1e-9
        mask[self.stop_action] = True
        return mask

    def _observe(self):
        rows = np.zeros((self.max_candidates, 8), dtype=np.float32)
        for i, candidate in enumerate(self.candidates):
            rows[i, :2] = (candidate.xy - self.origin) / self.span
            rows[i, 2] = candidate.confidence
            rows[i, 3 + candidate.kind] = 1.
            rows[i, -1] = candidate.visits
        return {
            "position": ((self.routes.xy[self.node] - self.origin) / self.span).astype(np.float32),
            "time_remaining": np.array([max(0., self.budget_s - self.elapsed)], dtype=np.float32),
            "candidates": rows,
            "mask": self.action_masks()[:-1].astype(np.int8),
        }

    def _info(self):
        return {"dirt_seed": self.dirt_seed, "dirt_at_start": self.remaining_at_start,
                "dirt_left": self.field.remaining, "extra_time_s": self.elapsed,
                "extra_distance_m": self.travelled, "false_recleans": self.false_visits,
                "visits": self.visits}

    def _visit(self, end):
        """Apply travel and a local brush stroke at one reachable destination."""
        duration = self._time_to(end)
        if self.elapsed + duration > self.budget_s + 1e-9:
            return None
        target = self.routes.xy[end]
        had_dirt = any(p.removed_at is None and math.dist(target, (p.x, p.y)) <= self.brush_radius
                       for p in self.field.patches)
        path = self.routes.path(self.node, end) + list(self.routes.strokes[end])
        distance = 0.
        removed = []
        last_xy = self.routes.xy[self.node]
        for xy in self.routes.xy[path]:
            distance += math.dist(last_xy, xy)
            removed.extend(self.field.update(*xy, self.elapsed + distance / self.speed))
            last_xy = xy
        self.node = end
        self.elapsed += duration
        self.travelled += distance
        self.false_visits += int(not had_dirt)
        self.visits += 1
        reward = sum(1.8 if p.stubborn else 1. for p in removed)
        return reward - .026 * duration - .25 * (not had_dirt)

    def step(self, action):
        if self.episode_over:
            raise RuntimeError("Reset before starting a new episode")
        valid = self.action_space.contains(action)
        if valid and action == self.stop_action:
            self.episode_over = True
            return self._observe(), 0., True, False, self._info()
        if not valid or not self.action_masks()[int(action)]:
            # A non-masked learner still gets a bounded result for invalid moves.
            self.episode_over = True
            return self._observe(), -1., True, False, self._info()
        candidate = self.candidates[int(action)]
        candidate.visits += 1
        reward = self._visit(candidate.node)
        self._order_candidates()
        truncated = self.visits >= self.max_visits or not self.action_masks()[:-1].any()
        self.episode_over = truncated
        return self._observe(), float(reward), False, truncated, self._info()

    def greedy_spots(self):
        """Existing map-gap set cover and ordering applied to the initial sweep."""
        plan = reclean.plan_reclean(self.grid, self.plan.masks, self.plan.params,
                                   self.sweep_covered, self.routes.xy[self.node])
        return [self.routes.nearest(piece.poses[0][:2]) for piece in plan.pieces]
