"""Shared runtime, experiment, evaluation, and recording configuration."""
import copy
import json
import math
from pathlib import Path


def load_runtime(config_dir=None):
    directory = Path(config_dir) if config_dir else Path(__file__).resolve().parents[1] / 'config'
    return json.loads((directory / 'runtime.json').read_text())


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
    files = ['experiment.yaml', 'costmap.yaml', 'controllers/common.yaml', 'controllers/reference.yaml',
             'controllers/dwal.yaml', 'command_pipeline.yaml', 'scan_filter.yaml', 'launch.yaml', 'simulation.yaml']
    for relative in files:
        source = path if relative == 'experiment.yaml' else directory / relative
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
    directory = Path(package_root) / 'config'
    defaults = yaml.safe_load((directory / 'scenario_defaults.yaml').read_text())
    exp = experiment if experiment is not None else yaml.safe_load((directory / 'experiment.yaml').read_text())['experiment']
    defaults['evaluation'] = {key: exp['evaluation_defaults'][key]
                              for key in ('goal_tolerance', 'personal_space')}
    return deep_merge(defaults, override)


def load_evaluation(directory=None):
    directory = Path(directory) if directory else Path(__file__).resolve().parents[1]/'config'
    cfg = json.loads((directory/'evaluation.json').read_text())
    for key in ('length_jump_threshold_m', 'reference_radius_m', 'reference_motion_threshold_mps',
                'duration_s', 'replay_start_sim_s', 'replay_max_lateness_s',
                'minimum_coverage_fraction', 'stationary_threshold_mps', 'contact_rate_hz', 'contact_max_gap_s', 'contact_release_s'):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    return cfg


def resolve_variant(controller, variant, cfg):
    variant = variant or ('fixed' if controller == 'fixed_dwal' else cfg['default_dynamic_variant'])
    if (controller, variant) not in {('fixed_dwal', 'fixed'),
                                   ('dynamic_dwal', 'dynamic_no_split'),
                                   ('dynamic_dwal', 'dynamic_split')}:
        raise ValueError(f'Invalid controller/variant: {controller}/{variant}')
    return variant


def recording_topics(exp, cfg, controller):
    t = {**cfg['topics'], **exp['topics']}
    keys = ['odom', 'scan', 'costmap', 'costmap_updates', 'published_footprint',
            'human_states', 'robot_states', 'tracked_agents_logging', 'tracked_agents_controller',
            'reference_cmd', 'selected_cmd', 'guarded_cmd', 'smoothed_cmd',
            'collision_checked_cmd', 'final_cmd', 'sampled_paths', 'sampled_markers',
            'clusters_far', 'far_markers', 'shared_path', 'human_path',
            'contacts', 'braking_requirement', 'far_diagnostics', 'guard_status']
    if controller == 'fixed_dwal':
        keys += ['clusters_near', 'near_markers', 'near_diagnostics']
    return list(dict.fromkeys(['/clock', '/tf', '/tf_static', '/parameter_events'] + [t[k] for k in keys]))
def main():
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

if __name__ == '__main__':
    main()
