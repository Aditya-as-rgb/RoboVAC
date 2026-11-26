import numpy as np
import pytest

from vacuum_rl import DetectorConfig, RecleanEnv
from vacuum_rl.env import Candidate
from vacuum_rl.shared.dirt_cleaner import DirtField, Patch


@pytest.mark.parametrize("training", [False, True])
def test_fixed_seed_repeats_episode(training):
    env = RecleanEnv(training=training)

    def run():
        obs, info = env.reset(seed=77)
        records = {(0, key): value for key, value in obs.items()}
        facts = [info]
        for i in range(1, 5):
            action = np.flatnonzero(env.action_masks())[0]
            obs, reward, terminated, truncated, info = env.step(action)
            records.update({(i, key): value for key, value in obs.items()})
            facts.append((reward, terminated, truncated, info))
            if terminated or truncated:
                break
        return records, facts

    first, facts = run()
    second, replay = run()
    assert facts == replay
    assert first.keys() == second.keys()
    for key in first:
        np.testing.assert_array_equal(first[key], second[key])


def test_padding_budget_and_stop_mask():
    env = RecleanEnv(budget_s=.1)
    obs, _ = env.reset(seed=3)
    assert env.candidates
    assert env.observation_space.contains(obs)
    assert not obs["candidates"][len(env.candidates):].any()
    assert not obs["mask"].any()
    assert np.flatnonzero(env.action_masks()).tolist() == [env.stop_action]
    _, reward, done, truncated, info = env.step(env.stop_action)
    assert done and not truncated and reward == 0.
    assert info["extra_time_s"] == info["false_recleans"] == 0


def test_invalid_action_does_not_move():
    env = RecleanEnv(detector=DetectorConfig(recall=(0.,) * 4))
    obs, before = env.reset(seed=3)
    assert not obs["candidates"].any()
    _, reward, done, truncated, after = env.step(0)
    assert done and not truncated and reward < 0.
    assert after == before


@pytest.mark.parametrize("dirt_present", [True, False])
def test_visit_reward_sign_and_counts(dirt_present):
    env = RecleanEnv(max_visits=1)
    env.reset(seed=3)
    xy = tuple(env.routes.xy[env.node])
    patches = [Patch("test", "crumbs", *xy)] if dirt_present else []
    env.field = DirtField(patches, env.brush_radius)
    env.field.update(*xy, 0.)
    env.candidates = [Candidate(env.node, xy, .8, 0)]
    assert env.action_masks()[0]
    obs, reward, done, truncated, info = env.step(0)
    assert not done and truncated
    assert (reward > 0) == dirt_present
    assert info["dirt_left"] == 0 and info["false_recleans"] == int(not dirt_present)
    assert info["extra_time_s"] > 0 and info["extra_distance_m"] > 0
    assert obs["candidates"][0, -1] == 1
