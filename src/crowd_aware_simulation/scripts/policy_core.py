"""Pure context-profile helpers shared by ROS nodes and unit tests."""


CONTEXT_LABELS = (
    'OPEN_AREA', 'NARROW_CORRIDOR', 'DOORWAY', 'JUNCTION',
    'DENSE_CROWD', 'OCCLUSION', 'UNKNOWN', 'STALE')


def choose_profile(current, candidate, scores, elapsed_since_transition,
                   minimum_dwell, switch_margin):
    """Hysteresis for ordinary transitions; UNKNOWN/STALE bypass dwell."""
    if candidate in ('STALE', 'UNKNOWN'):
        return candidate
    if current in ('STALE', 'UNKNOWN', ''):
        return candidate
    if candidate == current:
        return current
    if elapsed_since_transition < minimum_dwell:
        return current
    if scores.get(candidate, 0.0) < scores.get(current, 0.0) + switch_margin:
        return current
    return candidate
