"""Shortest-path travel on the reachable robot-centre grid."""

from functools import lru_cache
from itertools import product
import math

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree


class GridRoutes:
    def __init__(self, grid, reachable):
        self.cells = np.argwhere(reachable)
        self.xy = np.array([grid.cell_to_world(*cell) for cell in self.cells])
        self.tree = cKDTree(self.xy)
        self.resolution = grid.resolution
        ids = np.full(reachable.shape, -1, dtype=np.int32)
        ids[reachable] = np.arange(len(self.cells))
        origins, destinations, lengths = [], [], []
        for i, (row, col) in enumerate(self.cells):
            for dr, dc in product((-1, 0, 1), repeat=2):
                if (dr == dc == 0 or not grid.in_bounds(row + dr, col + dc)
                        or not reachable[row + dr, col + dc]):
                    continue
                if dr and dc and not (reachable[row + dr, col] and reachable[row, col + dc]):
                    continue
                origins.append(i)
                destinations.append(ids[row + dr, col + dc])
                lengths.append(grid.resolution * math.hypot(dr, dc))
        self.graph = csr_matrix((lengths, (origins, destinations)), shape=(len(self.cells),) * 2)
        self.from_node = lru_cache(maxsize=256)(self._from_node)
        self.strokes = tuple(self._stroke(i, ids) for i in range(len(self.cells)))

    def _from_node(self, node):
        return dijkstra(self.graph, indices=node, return_predecessors=True)

    def nearest(self, xy):
        return int(self.tree.query(xy)[1])

    def _stroke(self, node, ids):
        """A straight, reachable out-and-back brush stroke of up to 0.30 m."""
        row, col = self.cells[node]
        best = []
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            outward = []
            for i in range(1, 4):
                r, c = row + dr * i, col + dc * i
                if not (0 <= r < ids.shape[0] and 0 <= c < ids.shape[1]) or ids[r, c] < 0:
                    break
                outward.append(int(ids[r, c]))
            if len(outward) > len(best):
                best = outward
        return (*best, *reversed(best[:-1]), node) if best else ()

    def distance(self, start, end):
        return float(self.from_node(start)[0][end])

    def path(self, start, end):
        if start == end:
            return []
        distances, predecessors = self.from_node(start)
        if not np.isfinite(distances[end]):
            raise ValueError("Destination is disconnected from the robot")
        reverse = [end]
        while predecessors[reverse[-1]] != start:
            reverse.append(int(predecessors[reverse[-1]]))
        return list(reversed(reverse))

    def visit_distance(self, start, end):
        return self.distance(start, end) + self.resolution * len(self.strokes[end])
