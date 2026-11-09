"""Offline approximation of Nav2's Regulated Pure Pursuit, for checking plans.

This is NOT Nav2: it models the same idea (carrot at a fixed lookahead,
curvature = 2*y/L^2, slow down in tight turns, rotate in place when the carrot is
more than ~45 degrees off, slow near the goal) with a unicycle robot and a
controller that updates at a fixed rate. It exists so a plan can be checked
without Gazebo: how much floor does the trail clean, and how close does the
robot get to obstacles when the controller cuts corners.

Defaults mirror the CoveragePath controller in vacuum_bringup/config/navigation.yaml
(lookahead 0.25 m, 0.25 m/s, 5 Hz).
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class PursuitParams:
    desired_speed: float = 0.25
    lookahead: float = 0.25
    controller_dt: float = 0.2          # controller_frequency 5 Hz
    substep: float = 0.02
    rotate_min_angle: float = 0.785
    rotate_speed: float = 0.8
    min_radius: float = 0.5             # regulated_linear_scaling_min_radius
    min_speed: float = 0.10             # regulated_linear_scaling_min_speed
    approach_dist: float = 0.4
    min_approach_speed: float = 0.05
    goal_tolerance: float = 0.15
    max_angular: float = 2.0
    max_time: float = 900.0


def follow(path_xy, start_pose, p=None):
    """Drive a unicycle along ``path_xy`` ([(x, y)]); returns (trail, final_pose, time, ok).

    ``trail`` is a list of (x, y) samples, ``start_pose`` is (x, y, yaw).
    """
    p = p or PursuitParams()
    path = np.asarray(path_xy, dtype=float)
    x, y, th = start_pose
    trail = [(x, y)]
    t = 0.0
    idx = 0                                         # progress along the path
    goal = path[-1]
    while t < p.max_time:
        # advance the progress index to the closest point ahead
        window = path[idx: idx + 60]
        d = np.hypot(window[:, 0] - x, window[:, 1] - y)
        idx += int(np.argmin(d))
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)
        if dist_goal < p.goal_tolerance and idx >= len(path) - 8:
            return trail, (x, y, th), t, True
        # carrot: first pose from idx on that is at least `lookahead` away
        carrot = path[-1]
        for q in path[idx:]:
            if math.hypot(q[0] - x, q[1] - y) >= p.lookahead:
                carrot = q
                break
        dx, dy = carrot[0] - x, carrot[1] - y
        xl = math.cos(th) * dx + math.sin(th) * dy
        yl = -math.sin(th) * dx + math.cos(th) * dy
        ang = math.atan2(yl, xl)
        L2 = max(1e-6, xl * xl + yl * yl)
        if abs(ang) > p.rotate_min_angle and dist_goal > p.goal_tolerance:
            v, w = 0.0, math.copysign(p.rotate_speed, ang)
        else:
            kappa = 2.0 * yl / L2
            v = p.desired_speed
            if abs(kappa) > 1e-6 and (1.0 / abs(kappa)) < p.min_radius:
                v = max(p.min_speed, v * (1.0 / abs(kappa)) / p.min_radius)
            if dist_goal < p.approach_dist:
                v = max(p.min_approach_speed, v * dist_goal / p.approach_dist)
            w = max(-p.max_angular, min(p.max_angular, v * kappa))
        # hold the command for one controller period
        steps = int(round(p.controller_dt / p.substep))
        for _ in range(steps):
            x += v * math.cos(th) * p.substep
            y += v * math.sin(th) * p.substep
            th += w * p.substep
            trail.append((x, y))
        t += p.controller_dt
    return trail, (x, y, th), t, False


def run_plan(plan, start_pose, p=None):
    """Follow every piece in order, starting each from the robot's actual pose.

    Transit between pieces is not simulated (Nav2's planner does that on the real
    robot, and it also cleans on the way), so the coverage reported is a lower
    bound. Returns (trails, total_time_s, failures).
    """
    trails = []
    pose = start_pose
    total = 0.0
    failures = 0
    for piece in plan.pieces:
        path = [(q[0], q[1]) for q in piece.poses]
        # as if NavigateToPose had brought the robot to the first pose, facing along it
        pose = (path[0][0], path[0][1], piece.poses[0][2])
        trail, pose, t, ok = follow(path, pose, p)
        trails.append(trail)
        total += t
        failures += 0 if ok else 1
    return trails, total, failures


def min_clearance(points, grid, blocked):
    """Smallest distance (m) from any point to an occupied/unknown cell edge (conservative)."""
    res = grid.resolution
    rows, cols = np.nonzero(blocked)
    if len(rows) == 0:
        return float('inf')
    ox = grid.origin_x + (cols + 0.5) * res
    oy = grid.origin_y + (rows + 0.5) * res
    pts = np.asarray(points, dtype=float)
    best = float('inf')
    for i in range(0, len(pts), 400):
        chunk = pts[i:i + 400]
        d = np.hypot(chunk[:, None, 0] - ox[None, :], chunk[:, None, 1] - oy[None, :])
        best = min(best, float(d.min()))
    return best - 0.7071 * res
