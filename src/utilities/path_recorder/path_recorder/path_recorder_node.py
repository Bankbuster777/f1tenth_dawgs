#!/usr/bin/env python3
"""
Path Recorder Node

Records vehicle trajectory from /pf/pose/odom and compares it against the global path.
Calculates lateral error statistics (average, min, max) and visualizes the results.

Author: F1TENTH DAWGS
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

import numpy as np
import csv
import os
import math
from datetime import datetime
from collections import deque

# Import FrenetConverter from the frenet_conversion package
from frenet_conversion.frenet_converter import FrenetConverter


class PathRecorderNode(Node):
    """
    Records actual vehicle trajectory and compares with global path to compute lateral errors.
    """

    def __init__(self):
        super().__init__('path_recorder_node')

        # Declare parameters
        self.declare_parameter('odom_topic', '/pf/pose/odom')
        self.declare_parameter('global_path_topic', '/global_centerline')
        self.declare_parameter('recording_enabled', True)
        self.declare_parameter('save_to_csv', True)
        self.declare_parameter('csv_output_dir', os.path.expanduser('~/f1tenth_recordings'))
        self.declare_parameter('max_recording_points', 10000)  # Limit memory usage
        self.declare_parameter('stats_update_rate', 1.0)  # Hz

        # Visualization parameters
        self.declare_parameter('triangle_size', 0.15)  # Triangle size in meters
        self.declare_parameter('triangle_spacing', 0.5)  # Distance between triangles in meters
        self.declare_parameter('min_speed', 0.0)  # Min speed for color mapping (m/s)
        self.declare_parameter('max_speed', 8.0)  # Max speed for color mapping (m/s)

        # Get parameters
        self.odom_topic = self.get_parameter('odom_topic').value
        self.global_path_topic = self.get_parameter('global_path_topic').value
        self.recording_enabled = self.get_parameter('recording_enabled').value
        self.save_to_csv = self.get_parameter('save_to_csv').value
        self.csv_output_dir = self.get_parameter('csv_output_dir').value
        self.max_recording_points = self.get_parameter('max_recording_points').value
        stats_update_rate = self.get_parameter('stats_update_rate').value

        # Get visualization parameters
        self.triangle_size = self.get_parameter('triangle_size').value
        self.triangle_spacing = self.get_parameter('triangle_spacing').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value

        # State variables
        self.global_path_received = False
        self.frenet_converter = None
        self.recorded_positions = deque(maxlen=self.max_recording_points)
        self.lateral_errors = deque(maxlen=self.max_recording_points)

        # Statistics
        self.error_stats = {
            'count': 0,
            'sum': 0.0,
            'sum_sq': 0.0,
            'min': float('inf'),
            'max': float('-inf'),
            'avg': 0.0,
            'std': 0.0
        }

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.global_path_sub = self.create_subscription(
            Path,
            self.global_path_topic,
            self.global_path_callback,
            10
        )

        # Publishers
        self.recorded_path_marker_pub = self.create_publisher(
            Marker,
            '/path_recorder/recorded_path',
            10
        )

        self.error_visualization_pub = self.create_publisher(
            MarkerArray,
            '/path_recorder/error_visualization',
            10
        )

        # Timers
        self.stats_timer = self.create_timer(
            1.0 / stats_update_rate,
            self.publish_statistics
        )

        self.viz_timer = self.create_timer(
            0.2,  # 5 Hz
            self.publish_visualizations
        )

        # Create output directory if needed
        if self.save_to_csv:
            os.makedirs(self.csv_output_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_filename = os.path.join(
                self.csv_output_dir,
                f'path_recording_{timestamp}.csv'
            )
            self._initialize_csv()

        self.get_logger().info('='*60)
        self.get_logger().info('Path Recorder Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Odometry topic: {self.odom_topic}')
        self.get_logger().info(f'Global path topic: {self.global_path_topic}')
        self.get_logger().info(f'Recording enabled: {self.recording_enabled}')
        self.get_logger().info(f'Save to CSV: {self.save_to_csv}')
        if self.save_to_csv:
            self.get_logger().info(f'CSV output: {self.csv_filename}')
        self.get_logger().info('Waiting for global path...')

    def _initialize_csv(self):
        """Initialize CSV file with headers."""
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'timestamp',
                'x_m',
                'y_m',
                'vx_mps',
                'vy_mps',
                's_m',
                'd_m (lateral_error)',
                'abs_lateral_error'
            ])

    def global_path_callback(self, msg: Path):
        """Process incoming global path and initialize Frenet converter."""
        if self.global_path_received:
            return  # Only process once

        if len(msg.poses) < 3:
            self.get_logger().warn(f'Global path has only {len(msg.poses)} points. Need at least 3.')
            return

        # Extract waypoint data from nav_msgs/Path
        x_array = np.array([pose.pose.position.x for pose in msg.poses])
        y_array = np.array([pose.pose.position.y for pose in msg.poses])

        # Convert quaternions to yaw angles
        psi_array = np.array([self._quaternion_to_yaw(pose.pose.orientation) for pose in msg.poses])

        # Initialize Frenet converter
        try:
            self.frenet_converter = FrenetConverter(x_array, y_array, psi_array)
            self.global_path_received = True

            self.get_logger().info('='*60)
            self.get_logger().info('Global path received!')
            self.get_logger().info(f'Path length: {self.frenet_converter.raceline_length:.2f} m')
            self.get_logger().info(f'Number of waypoints: {len(msg.poses)}')
            self.get_logger().info('Now recording vehicle trajectory...')
            self.get_logger().info('='*60)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize FrenetConverter: {e}')

    def _quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        """Process incoming odometry and calculate lateral error."""
        if not self.recording_enabled or not self.global_path_received:
            return

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        try:
            # Convert to Frenet coordinates
            frenet_coords = self.frenet_converter.get_frenet([x], [y])
            s = frenet_coords[0, 0]
            d = frenet_coords[1, 0]  # This is the lateral error

            # Record data
            self.recorded_positions.append({
                'x': x,
                'y': y,
                'vx': vx,
                'vy': vy,
                's': s,
                'd': d,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            })

            self.lateral_errors.append(d)

            # Update statistics
            self._update_statistics(d)

            # Save to CSV if enabled
            if self.save_to_csv:
                self._append_to_csv({
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'x': x,
                    'y': y,
                    'vx': vx,
                    'vy': vy,
                    's': s,
                    'd': d,
                    'abs_d': abs(d)
                })

        except Exception as e:
            self.get_logger().error(f'Error processing odometry: {e}', throttle_duration_sec=1.0)

    def _update_statistics(self, lateral_error: float):
        """Update running statistics for lateral error."""
        self.error_stats['count'] += 1
        self.error_stats['sum'] += lateral_error
        self.error_stats['sum_sq'] += lateral_error ** 2
        self.error_stats['min'] = min(self.error_stats['min'], lateral_error)
        self.error_stats['max'] = max(self.error_stats['max'], lateral_error)

        # Calculate average and standard deviation
        n = self.error_stats['count']
        self.error_stats['avg'] = self.error_stats['sum'] / n
        variance = (self.error_stats['sum_sq'] / n) - (self.error_stats['avg'] ** 2)
        self.error_stats['std'] = np.sqrt(max(0, variance))  # Avoid negative due to numerical error

    def _append_to_csv(self, data: dict):
        """Append data to CSV file."""
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                data['timestamp'],
                data['x'],
                data['y'],
                data['vx'],
                data['vy'],
                data['s'],
                data['d'],
                data['abs_d']
            ])

    def publish_statistics(self):
        """Publish statistics to console."""
        if self.error_stats['count'] == 0:
            return

        stats = self.error_stats
        self.get_logger().info(
            f"Lateral Error Stats | "
            f"Count: {stats['count']} | "
            f"Avg: {stats['avg']*100:.2f} cm | "
            f"Std: {stats['std']*100:.2f} cm | "
            f"Min: {stats['min']*100:.2f} cm | "
            f"Max: {stats['max']*100:.2f} cm"
        )

    def publish_visualizations(self):
        """Publish visualization markers."""
        if not self.global_path_received or len(self.recorded_positions) == 0:
            return

        # Publish recorded path as line strip
        self._publish_recorded_path_marker()

        # Publish error visualization as colored arrows
        self._publish_error_markers()

    def _publish_recorded_path_marker(self):
        """Publish the recorded path as triangles with velocity-based coloring."""
        if len(self.recorded_positions) < 2:
            self.get_logger().debug(f'Not enough points to visualize: {len(self.recorded_positions)}', throttle_duration_sec=5.0)
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'recorded_path_triangles'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        # Scale (not used for TRIANGLE_LIST, but set for completeness)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Convert to list for easier indexing
        positions = list(self.recorded_positions)

        # Track cumulative distance for spacing
        cumulative_distance = 0.0
        last_added_distance = 0.0

        for i in range(len(positions) - 1):
            current = positions[i]
            next_pos = positions[i + 1]

            # Calculate distance from previous point
            dx = next_pos['x'] - current['x']
            dy = next_pos['y'] - current['y']
            segment_distance = np.sqrt(dx**2 + dy**2)
            cumulative_distance += segment_distance

            # Only add triangle if we've traveled enough distance
            if cumulative_distance - last_added_distance < self.triangle_spacing:
                continue

            last_added_distance = cumulative_distance

            # Calculate heading angle from velocity
            vx = current['vx']
            vy = current['vy']
            speed = np.sqrt(vx**2 + vy**2)

            # Skip if vehicle is nearly stationary
            if speed < 0.001:
                continue

            heading = np.arctan2(vy, vx)

            # Calculate triangle vertices
            # Tip points in direction of travel
            tip_x = current['x'] + self.triangle_size * np.cos(heading)
            tip_y = current['y'] + self.triangle_size * np.sin(heading)

            # Base of triangle perpendicular to heading
            base_width = self.triangle_size * 0.6
            base_left_x = current['x'] + base_width * np.cos(heading + np.pi / 2)
            base_left_y = current['y'] + base_width * np.sin(heading + np.pi / 2)
            base_right_x = current['x'] + base_width * np.cos(heading - np.pi / 2)
            base_right_y = current['y'] + base_width * np.sin(heading - np.pi / 2)

            # Create triangle vertices
            tip = Point()
            tip.x = tip_x
            tip.y = tip_y
            tip.z = 0.05

            base_left = Point()
            base_left.x = base_left_x
            base_left.y = base_left_y
            base_left.z = 0.05

            base_right = Point()
            base_right.x = base_right_x
            base_right.y = base_right_y
            base_right.z = 0.05

            # Add triangle (vertices must be in counter-clockwise order for proper rendering)
            marker.points.extend([tip, base_left, base_right])

            # Calculate color based on speed
            # Normalize speed to [0, 1] range
            normalized_speed = (speed - self.min_speed) / max(self.max_speed - self.min_speed, 0.1)
            normalized_speed = np.clip(normalized_speed, 0.0, 1.0)

            # Color gradient: Blue (low speed) -> Green -> Red (high speed)
            if normalized_speed < 0.5:
                # Blue to green
                r = 0.0
                g = 2.0 * normalized_speed
                b = 1.0 - 2.0 * normalized_speed
            else:
                # Green to red
                r = 2.0 * (normalized_speed - 0.5)
                g = 1.0 - 2.0 * (normalized_speed - 0.5)
                b = 0.0

            color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)

            # Add same color for all three vertices of the triangle
            marker.colors.extend([color, color, color])

        # Debug info
        num_triangles = len(marker.points) // 3
        if num_triangles == 0:
            self.get_logger().warn(
                f'No triangles generated! Total positions: {len(positions)}, '
                f'Triangle spacing: {self.triangle_spacing}m',
                throttle_duration_sec=5.0
            )
        else:
            self.get_logger().debug(
                f'Publishing {num_triangles} triangles from {len(positions)} recorded positions',
                throttle_duration_sec=10.0
            )

        self.recorded_path_marker_pub.publish(marker)

    def _publish_error_markers(self):
        """Publish error visualization as arrows from recorded path to global path."""
        marker_array = MarkerArray()

        # Sample points to avoid too many markers
        step = max(1, len(self.recorded_positions) // 100)  # Max 100 markers

        for i, pos in enumerate(list(self.recorded_positions)[::step]):
            # Create arrow marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'lateral_error'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Start point: actual position
            start = Point()
            start.x = pos['x']
            start.y = pos['y']
            start.z = 0.1

            # End point: projected position on global path
            try:
                xy_on_path = self.frenet_converter.get_cartesian(pos['s'], 0.0)
                end = Point()
                end.x = float(xy_on_path[0])
                end.y = float(xy_on_path[1])
                end.z = 0.1

                marker.points = [start, end]

                # Scale
                marker.scale.x = 0.02  # Shaft diameter
                marker.scale.y = 0.04  # Head diameter
                marker.scale.z = 0.05  # Head length

                # Color based on error magnitude (green = small, red = large)
                error_abs = abs(pos['d'])
                # Normalize error to [0, 1] range, assuming max error of 0.5m
                normalized_error = min(1.0, error_abs / 0.5)
                marker.color = ColorRGBA(
                    r=float(normalized_error),
                    g=float(1.0 - normalized_error),
                    b=0.0,
                    a=0.6
                )

                marker_array.markers.append(marker)
            except Exception as e:
                self.get_logger().error(
                    f'Error creating arrow marker: {e}',
                    throttle_duration_sec=5.0
                )

        self.error_visualization_pub.publish(marker_array)

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        if self.save_to_csv and self.error_stats['count'] > 0:
            self.get_logger().info('='*60)
            self.get_logger().info('Path Recorder Shutting Down')
            self.get_logger().info('='*60)
            self.get_logger().info(f'Total points recorded: {self.error_stats["count"]}')
            self.get_logger().info(f'Data saved to: {self.csv_filename}')

            # Save summary statistics to a separate file
            summary_file = self.csv_filename.replace('.csv', '_summary.txt')
            with open(summary_file, 'w') as f:
                f.write('Path Recording Summary\n')
                f.write('='*60 + '\n')
                f.write(f'Total points: {self.error_stats["count"]}\n')
                f.write(f'Average lateral error: {self.error_stats["avg"]*100:.2f} cm\n')
                f.write(f'Std deviation: {self.error_stats["std"]*100:.2f} cm\n')
                f.write(f'Min lateral error: {self.error_stats["min"]*100:.2f} cm\n')
                f.write(f'Max lateral error: {self.error_stats["max"]*100:.2f} cm\n')

            self.get_logger().info(f'Summary saved to: {summary_file}')
            self.get_logger().info('='*60)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PathRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
