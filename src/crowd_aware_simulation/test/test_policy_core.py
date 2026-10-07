import importlib.util
from pathlib import Path

path = Path(__file__).resolve().parents[1] / 'scripts' / 'policy_core.py'
spec = importlib.util.spec_from_file_location('policy_core', path)
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)



def test_stale_bypasses_dwell():
    scores = {'OPEN_AREA': 1.0, 'STALE': 1.0}
    assert policy.choose_profile('OPEN_AREA', 'STALE', scores, 0.0, 2.0, 0.1) == 'STALE'


def test_ordinary_transition_hysteresis():
    scores = {'OPEN_AREA': 0.4, 'DENSE_CROWD': 0.8}
    assert policy.choose_profile('OPEN_AREA', 'DENSE_CROWD', scores, 0.5, 2.0, 0.1) == 'OPEN_AREA'
    assert policy.choose_profile('OPEN_AREA', 'DENSE_CROWD', scores, 2.1, 2.0, 0.1) == 'DENSE_CROWD'
