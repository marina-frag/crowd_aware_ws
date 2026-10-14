#!/usr/bin/env python3
"""Small checked adaptation of the existing shared controller to variable radii."""
import argparse
from pathlib import Path
import re


def patch(source):
    if '// DWAL_VARIABLE_R_V2' in source:
        return source

    def replace(pattern, replacement):
        nonlocal source
        source, count = re.subn(pattern, lambda _: replacement, source)
        if count != 1:
            raise RuntimeError(f'Unexpected shared_controller.cpp: expected one match, got {count}: {pattern}')

    replace(r'const auto\s*&\s*clusters_msg\s*=\s*\*chosen\s*;',
            'auto clusters_msg = *chosen; // DWAL_VARIABLE_R_V2: private intent coordinates')
    replace(r'level_\s*=\s*clusters_msg\.clusters\[0\]\.r\s*;', '''const double reference_r = this->get_parameter("intent_reference_radius").as_double();
  level_ = reference_r > 0.0 ? reference_r : clusters_msg.clusters[0].r;
  if (reference_r > 0.0) {
    // Compare steering intent at one reference radius. Actual cluster radii stay variable.
    for (auto & cluster : clusters_msg.clusters)
      for (auto & path : cluster.paths)
        path.phi = cluster_lib::curv2phi(path.curvature, reference_r);
  }''')
    replace(r'double phi_star\s*=\s*0\.0\s*;',
            'double phi_star = 0.0;\n  double selected_curvature = 0.0;')
    replace(r'phi_star\s*=\s*phi\s*;',
            'phi_star = phi;\n      selected_curvature = p.curvature;')
    replace(r'double k_star\s*=\s*cluster_lib::phi2curv\(phi_star,\s*level_\)\s*;',
            'double k_star = reference_r > 0.0 ? selected_curvature : cluster_lib::phi2curv(phi_star, level_);')
    replace(r'publishIntentPlotAtClusterHorizonMeanPhi\(intent,\s*clusters_msg\);',
            'publishIntentPlotAtClusterHorizonMeanPhi(intent, *chosen);')
    replace(r'cluster_lib::generatePath\(shared_path,\s*k_star,\s*x0,\s*y0,\s*th0,\s*0\.2,\s*level_\s*\*\s*level_,\s*odom_frame_id_\);',
            '''const double selected_radius = reference_r > 0.0 ? chosen_cluster.r : level_;
  cluster_lib::generatePath(shared_path, k_star, x0, y0, th0, 0.2,
    selected_radius * selected_radius, odom_frame_id_);''')
    pattern = r'(Kmax_\s*=\s*this->declare_parameter<double>\("Kmax",[^;]+;)'
    matches = list(re.finditer(pattern, source))
    if len(matches) != 1:
        raise RuntimeError('Shared controller Kmax parameter declaration differs')
    at = matches[0].end()
    source = source[:at] + '''
  const double reference_radius = this->declare_parameter<double>("intent_reference_radius", 0.0);
  if (!std::isfinite(reference_radius) || reference_radius < 0.0)
    throw std::invalid_argument("intent_reference_radius must be finite and nonnegative");
''' + source[at:]
    return source



def patch_runtime_fix(source):
    if '// DWAL_RUNTIME_FIX_V3' in source:
        return source

    def replace(old, new):
        nonlocal source
        if source.count(old) != 1:
            raise RuntimeError('Unexpected shared controller for runtime repair: ' + old[:80])
        source = source.replace(old, new, 1)

    replace('  output_cmd_topic_ = this->declare_parameter<std::string>("output_cmd_topic", "/cmd_vel");',
        '  output_cmd_topic_ = this->declare_parameter<std::string>("output_cmd_topic", "/cmd_vel");' +
        '\n  // DWAL_RUNTIME_FIX_V3: this mode must feed the existing collision-monitor pipeline.' +
        '\n  const bool checked_inplace = this->declare_parameter<bool>("inplace_via_collision_monitor", false);' +
        '\n  if (checked_inplace && output_cmd_topic_ != "/cmd_vel_selected")' +
        '\n    throw std::invalid_argument("inplace_via_collision_monitor requires /cmd_vel_selected output");')
    replace('  const double wz_user_raw = last_user_cmd_.twist.angular.z;',
        '  const double wz_user_raw = last_user_cmd_.twist.angular.z;' +
        '\n  if (!std::isfinite(v_user_raw) || !std::isfinite(wz_user_raw)) {' +
        '\n    publishCmd(MakeZeroTwist()); return;' +
        '\n  }' +
        '\n  // Pure rotation does not require a forward arc-line cluster.' +
        '\n  // Freshness was checked above. Guard, smoother, footprint collision monitor' +
        '\n  // and final watchdog still check this command before Gazebo receives it.' +
        '\n  if (this->get_parameter("inplace_via_collision_monitor").as_bool() &&' +
        '\n      std::fabs(v_user_raw) < 1.e-3) {' +
        '\n    auto rotation = MakeZeroTwist();' +
        '\n    rotation.twist.angular.z = wz_user_raw;' +
        '\n    publishCmd(rotation);' +
        '\n    return;' +
        '\n  }')
    replace('      RCLCPP_WARN_SKIPFIRST(this->get_logger(),\n        "No valid clusters. Outputting controlled stop.");',
        '      RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,\n'
        '        "No valid clusters. Outputting controlled stop.");')
    # The safety patch already guards singleton phi spans; require that protection.
    if 'double phi_span = std::max(std::fabs(' not in source:
        raise RuntimeError('Shared controller lacks the singleton span guard from the safety patch')
    return source


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('package', type=Path)
    args = parser.parse_args()
    sources = list(args.package.rglob('shared_controller.cpp'))
    if len(sources) != 1:
        raise SystemExit('Expected exactly one shared_controller.cpp in the dependency')
    source = sources[0]
    before = source.read_text()
    after = patch_runtime_fix(patch(before))  # All checks finish before writing anything.
    if before != after:
        source.write_text(after)
    print(f'DWAL_RUNTIME_FIX_V3 shared-controller adaptation ready: {source}')


if __name__ == '__main__':
    main()
