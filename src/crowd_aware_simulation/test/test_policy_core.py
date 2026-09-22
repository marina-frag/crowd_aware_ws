import importlib.util
from pathlib import Path

path = Path(__file__).resolve().parents[1] / 'scripts' / 'policy_core.py'
spec = importlib.util.spec_from_file_location('policy_core', path)
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


def test_horizon_nominal():
    result = policy.horizon_policy(0.2, 0.865, 0.2, 0.35, 0.5, 2.5,
                                   1.2, 8.0, 4.0, 0.3)
    assert result.valid
    assert 1.2 <= result.applied <= 4.0
    assert result.speed_limit == 0.3


def test_horizon_infeasible_reduces_speed():
    result = policy.horizon_policy(2.0, 0.865, 0.4, 0.7, 0.5, 4.0,
                                   1.2, 8.0, 2.0, 2.0)
    assert result.valid
    assert result.requested > result.upper
    assert result.speed_limit < 2.0
    assert 'speed reduced' in result.reason or result.speed_limit == 0.0


def test_invalid_braking_stops():
    result = policy.horizon_policy(0.2, 0.865, 0.2, 0.35, 0.0, 2.5,
                                   1.2, 8.0, 4.0, 0.3)
    assert not result.valid
    assert result.speed_limit == 0.0


def test_stale_bypasses_dwell():
    scores = {'OPEN_AREA': 1.0, 'STALE': 1.0}
    assert policy.choose_profile('OPEN_AREA', 'STALE', scores, 0.0, 2.0, 0.1) == 'STALE'


def test_ordinary_transition_hysteresis():
    scores = {'OPEN_AREA': 0.4, 'DENSE_CROWD': 0.8}
    assert policy.choose_profile('OPEN_AREA', 'DENSE_CROWD', scores, 0.5, 2.0, 0.1) == 'OPEN_AREA'
    assert policy.choose_profile('OPEN_AREA', 'DENSE_CROWD', scores, 2.1, 2.0, 0.1) == 'DENSE_CROWD'


def test_discrete_radius_is_conservative():
    assert policy.snap_radius(1.4, [1.2, 1.8, 2.6]) == 1.8
