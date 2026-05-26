#!/usr/bin/env python3
"""Record waypoints by sampling stable map->base_footprint pose."""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import yaml


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.yaml_path = 'waypoints.yaml'
        self.waypoints = {}
        self._load_existing()

    def _load_existing(self):
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
                self.waypoints = data.get('waypoints', {})
                n = len(self.waypoints)
                if n > 0:
                    self.get_logger().info(f'Loaded {n} existing waypoints from {self.yaml_path}')
        except FileNotFoundError:
            self.waypoints = {}

    def _save(self):
        data = {'waypoints': self.waypoints}
        with open(self.yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f'Saved waypoints to {self.yaml_path}')

    def _sample_pose(self, duration_sec=3.0, rate_hz=20):
        xs, ys, zs, ws = [], [], [], []
        dt = 1.0 / rate_hz
        start = time.time()
        while time.time() - start < duration_sec:
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time()
                )
                t = trans.transform.translation
                r = trans.transform.rotation
                xs.append(t.x)
                ys.append(t.y)
                zs.append(r.z)
                ws.append(r.w)
            except Exception:
                pass
            time.sleep(dt)

        if not xs:
            return None

        xs.sort()
        ys.sort()
        zs.sort()
        ws.sort()
        n = len(xs)
        mid = n // 2
        if n % 2 == 1:
            return [xs[mid], ys[mid], zs[mid], ws[mid]]
        return [
            (xs[mid - 1] + xs[mid]) / 2,
            (ys[mid - 1] + ys[mid]) / 2,
            (zs[mid - 1] + zs[mid]) / 2,
            (ws[mid - 1] + ws[mid]) / 2,
        ]

    def run(self):
        self.get_logger().info('Waypoint recorder ready.')
        self.get_logger().info('Enter A/B/C/D/E to record, q to quit.')
        while rclpy.ok():
            try:
                cmd = input('> ').strip().upper()
            except EOFError:
                break
            if cmd == 'Q':
                break
            if cmd in 'ABCDE':
                self.get_logger().info(
                    f'Sampling pose for {cmd} ({3}s, keep still)...'
                )
                pose = self._sample_pose()
                if pose is None:
                    self.get_logger().error(
                        'Failed to sample pose. Is AMCL running?'
                    )
                    continue
                self.waypoints[cmd] = [float(f'{v:.6f}') for v in pose]
                self.get_logger().info(f'{cmd}: {self.waypoints[cmd]}')
                self._save()
            else:
                self.get_logger().warn('Use A/B/C/D/E or q.')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
