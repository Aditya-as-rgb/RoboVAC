"""Train masked PPO and benchmark paired, held-out dirt layouts."""

import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
from statistics import mean, stdev

import numpy as np

from .env import DetectorConfig, RecleanEnv, TRAIN_DIRT_SEEDS
from .shared.dirt_cleaner import chord_length

METRICS = ("dirt_left", "extra_time_s", "extra_distance_m", "false_recleans")


def train(output, steps, seed):
    import torch
    from sb3_contrib import MaskablePPO
    from stable_baselines3.common.monitor import Monitor

    torch.set_num_threads(1)
    env = Monitor(RecleanEnv(training=True))
    model = MaskablePPO("MultiInputPolicy", env, device="cpu", seed=seed, verbose=1,
                        n_steps=512, batch_size=128, n_epochs=5, ent_coef=.01)
    model.learn(total_timesteps=steps)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(output))
    env.close()


def _oracle_action(env):
    """Prioritize attainable brush progress per second with perfect detections."""
    scores = []
    mask = env.action_masks()
    for i, candidate in enumerate(env.candidates):
        if not mask[i]:
            continue
        end = candidate.node
        stroke = env.routes.xy[[end, *env.routes.strokes[end]]]
        live = [p for p in env.field.patches if p.removed_at is None
                and np.linalg.norm(env.routes.xy[end] - [p.x, p.y]) < env.brush_radius]
        progress = 0.
        for p in live:
            scrub = sum(chord_length(a, b, (p.x, p.y), env.brush_radius)
                        for a, b in zip(stroke[:-1], stroke[1:]))
            if scrub > 0:
                fraction = min(1., scrub / (p.scrub_m - p.scrubbed))
                progress += (1.8 if p.stubborn else 1.) * fraction
        if progress:
            scores.append((progress / env._time_to(end), i))
    return max(scores)[1] if scores else env.stop_action


def _run(env, seed, policy, model):
    obs, _ = env.reset(seed=seed)
    reward = 0.
    if policy == "greedy":
        # The original map-based planner proposes goals regardless of detector output.
        for node in env.greedy_spots():
            if env.visits >= env.max_visits:
                break
            gain = env._visit(node)
            if gain is not None:
                reward += gain
        return {**env._info(), "reward": reward}
    rng = np.random.default_rng(seed + 1_000_000)
    while True:
        if policy == "none":
            action = env.stop_action
        elif policy == "random":
            action = rng.choice(np.flatnonzero(env.action_masks()))
        elif policy == "oracle":
            action = _oracle_action(env)
        else:
            action, _ = model.predict(obs, deterministic=True, action_masks=env.action_masks())
        obs, gain, terminated, truncated, info = env.step(action)
        reward += gain
        if terminated or truncated:
            return {**info, "reward": reward}


def evaluate(model_path, output, first_seed, count):
    import torch
    from sb3_contrib import MaskablePPO

    if first_seed < TRAIN_DIRT_SEEDS or count < 2:
        raise ValueError("Use at least two evaluation seeds outside the training range")
    torch.set_num_threads(1)
    model = MaskablePPO.load(model_path, device="cpu")
    standard = RecleanEnv()
    oracle = RecleanEnv(detector=DetectorConfig.perfect())
    policies = ("none", "greedy", "oracle", "random", "ppo")
    rows = []
    for seed in range(first_seed, first_seed + count):
        for policy in policies:
            result = _run(oracle if policy == "oracle" else standard, seed, policy, model)
            rows.append({"policy": policy, **result})

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "episodes.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "held_out_seeds": [first_seed, first_seed + count - 1],
        "episodes_per_policy": count,
        "ppo_seed": model.seed,
        "ppo_steps": model.num_timesteps,
        "dirt_count": standard.dirt_count,
        "reclean_budget_s": standard.budget_s,
        "detector": asdict(standard.detector),
        "policies": {},
    }
    for policy in policies:
        data = [row for row in rows if row["policy"] == policy]
        stats = {}
        for metric in METRICS:
            values = [row[metric] for row in data]
            stats[metric] = {"mean": mean(values), "std": stdev(values)}
        summary["policies"][policy] = stats
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    training = commands.add_parser("train")
    training.add_argument("--output", required=True)
    training.add_argument("--steps", type=int, default=32768)
    training.add_argument("--seed", type=int, default=42)
    evaluation = commands.add_parser("evaluate")
    evaluation.add_argument("--model", required=True)
    evaluation.add_argument("--output", required=True)
    evaluation.add_argument("--first-seed", type=int, default=10000)
    evaluation.add_argument("--count", type=int, default=64)
    args = parser.parse_args()
    if args.command == "train":
        train(args.output, args.steps, args.seed)
    else:
        evaluate(args.model, args.output, args.first_seed, args.count)


if __name__ == "__main__":
    main()
