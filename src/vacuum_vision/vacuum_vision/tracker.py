"""Merge repeated sightings of the same dirt patch into spots (no ROS).

A single frame is noisy: the same patch is seen many times from different places, and a
wall or a rug pattern can look like dirt for one frame. A spot becomes *confirmed* after
``min_hits`` frames. A confirmed spot is dropped ("cleared") when the camera looks straight
at its position from close range for ``clear_misses`` frames in a row and the detector finds
nothing there, which is what lets the robot see that its own cleaning worked.
"""

from dataclasses import dataclass, field
import math


@dataclass
class Spot:
    id: int
    x: float
    y: float
    hits: int = 0
    conf_sum: float = 0.0
    weight: float = 0.0
    misses: int = 0
    first_seen: float = 0.0
    last_seen: float = 0.0
    votes: dict = field(default_factory=dict)

    @property
    def kind(self):
        return max(self.votes, key=self.votes.get) if self.votes else 'dirt'

    @property
    def conf(self):
        return self.conf_sum / self.hits if self.hits else 0.0


class SpotTracker:
    def __init__(self, merge_radius=0.15, min_hits=3, clear_misses=10, unconfirmed_ttl=10.0):
        self.merge_radius = merge_radius
        self.min_hits = min_hits
        self.clear_misses = clear_misses
        self.unconfirmed_ttl = unconfirmed_ttl
        self.spots = []
        self._next_id = 1

    def is_confirmed(self, spot):
        return spot.hits >= self.min_hits

    def confirmed(self):
        return [s for s in self.spots if self.is_confirmed(s)]

    def reset(self):
        self.spots = []

    def _nearest(self, x, y):
        best, best_d = None, self.merge_radius
        for s in self.spots:
            d = math.hypot(s.x - x, s.y - y)
            if d <= best_d:
                best, best_d = s, d
        return best

    def update(self, sightings, t, in_view=None):
        """Feed one frame.

        sightings  list of (x, y, kind, conf) in the map frame
        t          frame time (s)
        in_view    optional f(x, y) -> bool: True if this frame's camera would see a patch
                   at (x, y) well enough that "not detected" means "not there"
        Returns the spots removed this frame because they were cleaned.
        """
        matched = set()
        for x, y, kind, conf in sorted(sightings, key=lambda s: -s[3]):
            s = self._nearest(x, y)
            if s is None:
                s = Spot(self._next_id, x, y, first_seen=t)
                self._next_id += 1
                self.spots.append(s)
            else:
                w = s.weight + conf
                s.x = (s.x * s.weight + x * conf) / w
                s.y = (s.y * s.weight + y * conf) / w
            s.weight += conf
            s.votes[kind] = s.votes.get(kind, 0.0) + conf
            if s.id not in matched:                    # two boxes on one patch count once per frame
                s.hits += 1
                s.conf_sum += conf
                matched.add(s.id)
            s.misses = 0
            s.last_seen = t
        cleared, kept = [], []
        for s in self.spots:
            if s.id in matched:
                kept.append(s)
            elif self.is_confirmed(s):
                if in_view is not None and in_view(s.x, s.y):
                    s.misses += 1
                if s.misses >= self.clear_misses:
                    cleared.append(s)
                    continue
                kept.append(s)
            elif t - s.last_seen <= self.unconfirmed_ttl:
                kept.append(s)                          # unconfirmed and not yet stale
            # else: one-off false positive, forgotten
        self.spots = kept
        return cleared
