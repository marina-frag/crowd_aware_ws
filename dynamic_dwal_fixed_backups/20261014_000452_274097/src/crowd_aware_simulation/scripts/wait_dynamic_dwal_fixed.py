#!/usr/bin/env python3
"""Wait for the parameter service used by the unchanged DWAL generator."""
import time
import rclpy
from rcl_interfaces.srv import GetParameters


def main():
    rclpy.init()
    node = rclpy.create_node('wait_dynamic_dwal_fixed')
    client = node.create_client(
        GetParameters, '/dwal_planner/dwal_clustering/get_parameters')
    deadline = time.monotonic() + 30.0
    code = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.25):
                request = GetParameters.Request()
                request.names = ['dwal_clustering/Marker_num']
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                if future.done() and future.exception() is None:
                    response = future.result()
                    if response and response.values and response.values[0].type == 2:
                        code = 0
                        break
        if code:
            node.get_logger().error('dynamic_dwal_fixed parameter service was not ready')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
