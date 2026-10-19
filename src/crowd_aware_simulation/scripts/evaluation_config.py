"""One evaluation/recording configuration, with no ROS imports."""
import json
import math
from pathlib import Path


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
