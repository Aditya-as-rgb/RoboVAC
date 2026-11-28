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

METRICS = ("dirt_left", "dirt_removed_frac", "extra_time_s", "extra_distance_m",
           "false_recleans", "reward")
PAIRED = ("dirt_left", "extra_time_s", "false_recleans", "reward")
REFERENCE = "heuristic"  # every other policy is compared against this one, seed by seed


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
    """Prioritize attainable brush progress per second (myopic, reads hidden dirt state)."""
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


def _heuristic_action(env, threshold):
    """Deployable baseline: visit each unvisited detection above `threshold`, nearest first.

    The env keeps candidates sorted nearest-first, so the first match is the nearest one.
    Skipping visited candidates matters: otherwise the robot re-visits the spot it stands on.
    """
    mask = env.action_masks()
    for i, candidate in enumerate(env.candidates):
        if mask[i] and candidate.visits == 0 and candidate.confidence >= threshold:
            return i
    return env.stop_action


def _run(env, seed, policy, model, threshold=.4):
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
        elif policy in ("oracle", "oracle_detected"):
            action = _oracle_action(env)
        elif policy == "heuristic":
            action = _heuristic_action(env, threshold)
        else:
            action, _ = model.predict(obs, deterministic=True, action_masks=env.action_masks())
        obs, gain, terminated, truncated, info = env.step(action)
        reward += gain
        if terminated or truncated:
            return {**info, "reward": reward}


def _paired_ci(a, b, rng, resamples=4000):
    """Mean of (a - b) over seeds with a bootstrap 95% interval."""
    diff = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    boots = diff[rng.integers(0, len(diff), (resamples, len(diff)))].mean(axis=1)
    low, high = np.percentile(boots, [2.5, 97.5])
    return {"mean_diff": float(diff.mean()), "ci95": [float(low), float(high)]}


def evaluate(model_path, output, first_seed, count, threshold=.4):
    if first_seed < TRAIN_DIRT_SEEDS or count < 2:
        raise ValueError("Use at least two evaluation seeds outside the training range")
    model = None
    policies = ["none", "greedy", REFERENCE, "oracle_detected", "oracle", "random"]
    if model_path:
        import torch
        from sb3_contrib import MaskablePPO

        torch.set_num_threads(1)
        model = MaskablePPO.load(model_path, device="cpu")
        policies.append("ppo")
    standard = RecleanEnv()
    perfect = RecleanEnv(detector=DetectorConfig.perfect())
    rows = []
    for seed in range(first_seed, first_seed + count):
        for policy in policies:
            result = _run(perfect if policy == "oracle" else standard, seed, policy, model,
                          threshold)
            start = result["dirt_at_start"]
            result["dirt_removed_frac"] = (1. - result["dirt_left"] / start) if start else ""
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
        "dirt_count": standard.dirt_count,
        "reclean_budget_s": standard.budget_s,
        "heuristic_threshold": threshold,
        "detector": asdict(standard.detector),
        "notes": {
            "oracle": "perfect detections, myopic progress-per-second ordering",
            "oracle_detected": "same ordering on the noisy detections (recall-limited ceiling)",
            "dirt_removed_frac": "mean over episodes that had dirt left after the sweep",
            "paired": f"policy minus {REFERENCE}, same seeds, bootstrap 95% interval",
        },
        "policies": {},
        "paired_vs_" + REFERENCE: {},
    }
    if model is not None:
        summary["ppo_seed"], summary["ppo_steps"] = model.seed, model.num_timesteps
    by_policy = {p: [row for row in rows if row["policy"] == p] for p in policies}
    for policy, data in by_policy.items():
        stats = {}
        for metric in METRICS:
            values = [row[metric] for row in data if row[metric] != ""]
            stats[metric] = {"mean": mean(values), "std": stdev(values) if len(values) > 1 else 0.}
        summary["policies"][policy] = stats
    rng = np.random.default_rng(0)
    for policy in policies:
        if policy != REFERENCE:
            summary["paired_vs_" + REFERENCE][policy] = {
                metric: _paired_ci([r[metric] for r in by_policy[policy]],
                                   [r[metric] for r in by_policy[REFERENCE]], rng)
                for metric in PAIRED}
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    print(f"{'policy':16}{'left':>7}{'removed':>9}{'time s':>8}{'false':>7}{'reward':>8}")
    for policy, stats in summary["policies"].items():
        print(f"{policy:16}{stats['dirt_left']['mean']:7.2f}"
              f"{stats['dirt_removed_frac']['mean']:9.2f}{stats['extra_time_s']['mean']:8.1f}"
              f"{stats['false_recleans']['mean']:7.2f}{stats['reward']['mean']:8.2f}")
    print(f"\npaired vs {REFERENCE} (mean difference, 95% CI):")
    for policy, metrics in summary["paired_vs_" + REFERENCE].items():
        text = "  ".join(f"{m}={v['mean_diff']:+.2f} [{v['ci95'][0]:+.2f},{v['ci95'][1]:+.2f}]"
                         for m, v in metrics.items())
        print(f"  {policy:16}{text}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    training = commands.add_parser("train")
    training.add_argument("--output", required=True)
    training.add_argument("--steps", type=int, default=32768)
    training.add_argument("--seed", type=int, default=42)
    evaluation = commands.add_parser("evaluate")
    evaluation.add_argument("--model", help="MaskablePPO .zip; omit to benchmark baselines only")
    evaluation.add_argument("--output", required=True)
    evaluation.add_argument("--first-seed", type=int, default=10000)
    evaluation.add_argument("--count", type=int, default=64)
    evaluation.add_argument("--threshold", type=float, default=.4,
                            help="confidence cut-off for the heuristic baseline")
    args = parser.parse_args()
    if args.command == "train":
        train(args.output, args.steps, args.seed)
    else:
        evaluate(args.model, args.output, args.first_seed, args.count, args.threshold)


if __name__ == "__main__":
    main()
