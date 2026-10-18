"""Configuration loading shared by the scene generator and launch.

The runtime CLI uses only the standard library so Docker startup does not
require PyYAML on the host. Full YAML loading runs in the ROS environment.
"""
import copy
import json
from pathlib import Path
import sys


def load_runtime(config_dir=None):
    directory = Path(config_dir) if config_dir else Path(__file__).resolve().parents[1]/'config'
    return json.loads((directory/'runtime.json').read_text())


def deep_merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_configuration(experiment_path):
    import yaml
    path = Path(experiment_path)
    directory = path.parent
    cfg = {'runtime': load_runtime(directory)}
    files = ['experiment.yaml', 'costmap.yaml', 'controllers/common.yaml', 'controllers/reference.yaml', 'controllers/dwal.yaml', 'command_pipeline.yaml', 'scan_filter.yaml', 'launch.yaml', 'simulation.yaml']
    for relative in files:
        source = path if relative == 'experiment.yaml' else directory/relative
        data = yaml.safe_load(source.read_text())
        duplicate = set(cfg).intersection(data)
        if duplicate:
            raise ValueError(f'{source}: duplicate top-level configuration sections: {duplicate}')
        cfg.update(data)
    exp = cfg['experiment']
    exp['context'].setdefault('odom_stale_after', exp['timeouts']['odom'])
    exp['context'].setdefault('track_stale_after', exp['timeouts']['tracks'])
    for name, profile in exp['profiles'].items():
        if not 0 <= profile['max_speed'] <= exp['limits']['max_linear']:
            raise ValueError(f'profiles.{name}.max_speed must be within global limits')
    return cfg


def load_scenario_defaults(package_root, override, experiment=None):
    import yaml
    directory = Path(package_root)/'config'
    defaults = yaml.safe_load((directory/'scenario_defaults.yaml').read_text())
    exp = experiment if experiment is not None else yaml.safe_load((directory/'experiment.yaml').read_text())['experiment']
    defaults['evaluation'] = {
        key: exp['evaluation_defaults'][key] for key in ('goal_tolerance', 'personal_space')}
    return deep_merge(defaults, override)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('key')
    parser.add_argument('--config-dir')
    args = parser.parse_args()
    value = load_runtime(args.config_dir)
    for part in args.key.split('.'):
        value = value[part]
    if isinstance(value, bool):
        print(str(value).lower())
    elif isinstance(value, list):
        print('\n'.join(map(str, value)))
    else:
        print(value)
