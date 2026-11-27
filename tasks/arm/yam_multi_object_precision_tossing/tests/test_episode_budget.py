"""A diagnostic action cap must not create or advance a second world."""
import inspect

import arena_episode as episode


class World:
    instances = []

    def __init__(self, seed, record=False):
        self.seed = seed
        self.actions = []
        self.closed = False
        self.instances.append(self)

    def observation(self):
        return {'rgbd': self.seed}

    def step(self, action):
        self.actions.append(action)

    def result(self):
        return {'actions': len(self.actions)}

    def close(self):
        self.closed = True


class Lock:
    def due(self):
        return False


class Solver:
    def __init__(self):
        self.observations = []

    def act(self, turn, observation):
        self.observations.append(observation)
        return {'turn': turn}


def test_single_set_has_exact_cap_and_one_world(monkeypatch):
    assert 'max_actions' in inspect.signature(episode.run_episode).parameters
    World.instances = []
    monkeypatch.setattr(episode, 'ArenaRound', World)
    solver = Solver()
    result = episode.run_episode(17, None, solver, Lock(), max_actions=14)
    assert result == {'actions': 14}
    assert len(World.instances) == 1 and World.instances[0].closed
    assert solver.observations == [{'rgbd': 17}] * 14


def test_explicit_reference_observer_does_not_change_public_default(monkeypatch):
    assert 'observe' in inspect.signature(episode.run_episode).parameters
    World.instances = []
    monkeypatch.setattr(episode, 'ArenaRound', World)
    solver = Solver()
    episode.run_episode(17, None, solver, Lock(), max_actions=1,
                        observe=lambda world: {'reference': world.seed})
    assert solver.observations == [{'reference': 17}]
    assert World.instances[0].closed
