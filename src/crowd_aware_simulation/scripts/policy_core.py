"""Pure policy helpers shared by ROS nodes and unit tests."""
from dataclasses import dataclass
import math


CONTEXT_LABELS = (
    'OPEN_AREA', 'NARROW_CORRIDOR', 'DOORWAY', 'JUNCTION',
    'DENSE_CROWD', 'OCCLUSION', 'UNKNOWN', 'STALE')


@dataclass(frozen=True)
class HorizonResult:
    valid: bool
    requested: float
    applied: float
    upper: float
    speed_limit: float
    reason: str


def horizon_policy(speed, front_extent, margin, reaction_time, brake_deceleration,
                   preview_time, radius_min, sensor_reliable, costmap_reliable,
                   configured_max_speed):
    values = (speed, front_extent, margin, reaction_time, brake_deceleration,
              preview_time, radius_min, sensor_reliable, costmap_reliable,
              configured_max_speed)
    if not all(math.isfinite(v) for v in values):
        return HorizonResult(False, math.nan, math.nan, math.nan, 0.0,
                             'non-finite horizon input')
    if brake_deceleration <= 0.0:
        return HorizonResult(False, math.nan, math.nan, math.nan, 0.0,
                             'brake_deceleration must be positive')
    if radius_min <= 0.0 or sensor_reliable <= 0.0 or costmap_reliable <= 0.0:
        return HorizonResult(False, math.nan, math.nan, math.nan, 0.0,
                             'radius bounds must be positive')
    upper = min(sensor_reliable, costmap_reliable)
    if upper < radius_min or upper <= front_extent + margin:
        return HorizonResult(False, math.nan, math.nan, upper, 0.0,
                             'reliable coverage cannot contain minimum stopping envelope')

    v = max(0.0, speed)
    r_stop = front_extent + margin + v * reaction_time + v * v / (2.0 * brake_deceleration)
    r_preview = v * preview_time
    requested = max(r_stop, r_preview)
    applied = min(max(requested, radius_min), upper)
    speed_limit = max(0.0, configured_max_speed)
    reason = 'within reliable coverage'

    if requested > upper:
        # Solve v^2/(2a) + tau*v + (extent+margin-upper) <= 0.
        discriminant = (brake_deceleration * reaction_time) ** 2 + 2.0 * brake_deceleration * (upper - front_extent - margin)
        braking_limit = 0.0 if discriminant <= 0.0 else -brake_deceleration * reaction_time + math.sqrt(discriminant)
        preview_limit = upper / preview_time if preview_time > 0.0 else math.inf
        speed_limit = min(speed_limit, max(0.0, braking_limit), max(0.0, preview_limit))
        reason = 'required horizon exceeds coverage; speed reduced'
        if speed_limit <= 1e-3:
            speed_limit = 0.0
            reason = 'required horizon infeasible; stop'

    return HorizonResult(True, requested, applied, upper, speed_limit, reason)


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


def snap_radius(requested, levels):
    ordered = sorted(float(v) for v in levels if math.isfinite(float(v)) and float(v) > 0.0)
    if not ordered:
        raise ValueError('discrete levels must contain positive finite values')
    for level in ordered:
        if level >= requested:
            return level
    return ordered[-1]
