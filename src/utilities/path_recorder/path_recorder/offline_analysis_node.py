#!/usr/bin/env python3
"""
Offline Analysis Node for Path Recorder

Analyzes recorded trajectory CSV files and compares them with global centerline CSV file.
Generates comprehensive matplotlib visualizations and statistics with map overlay.

Author: F1TENTH DAWGS
"""

import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import os
import glob
from datetime import datetime
import sys
import yaml
from PIL import Image


class OfflineAnalysisNode(Node):
    """
    Offline analysis node for comparing recorded trajectories with global path.
    """

    def __init__(self):
        super().__init__('offline_analysis_node')

        # Declare parameters
        self.declare_parameter('csv_directory', os.path.expanduser('~/f1tenth_recordings'))
        self.declare_parameter('csv_file', '')
        self.declare_parameter('global_path_csv', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('output_directory', os.path.expanduser('~/f1tenth_analysis'))
        self.declare_parameter('save_plots', True)
        self.declare_parameter('show_plots', True)

        # Get parameters
        self.csv_directory = self.get_parameter('csv_directory').value
        self.csv_file = self.get_parameter('csv_file').value
        self.global_path_csv = self.get_parameter('global_path_csv').value
        self.map_yaml = self.get_parameter('map_yaml').value
        self.output_directory = self.get_parameter('output_directory').value
        self.save_plots = self.get_parameter('save_plots').value
        self.show_plots = self.get_parameter('show_plots').value

        # Create output directory if it doesn't exist
        if self.save_plots:
            os.makedirs(self.output_directory, exist_ok=True)

        # Storage
        self.global_path = None
        self.map_image = None
        self.map_info = None

        self.get_logger().info('='*80)
        self.get_logger().info('Offline Analysis Node Started')
        self.get_logger().info('='*80)

        # Run analysis
        self.run_analysis()

    def load_map(self):
        """Load map image and yaml file."""
        if not self.map_yaml:
            self.get_logger().warn('No map YAML file specified, skipping map overlay')
            return False

        map_yaml_file = os.path.expanduser(self.map_yaml)

        if not os.path.exists(map_yaml_file):
            self.get_logger().error(f'Map YAML file not found: {map_yaml_file}')
            return False

        try:
            # Read YAML file
            with open(map_yaml_file, 'r') as f:
                map_data = yaml.safe_load(f)

            # Get map image path (relative to yaml file location)
            map_dir = os.path.dirname(map_yaml_file)
            map_image_path = os.path.join(map_dir, map_data['image'])

            if not os.path.exists(map_image_path):
                self.get_logger().error(f'Map image not found: {map_image_path}')
                return False

            # Load image
            self.map_image = np.array(Image.open(map_image_path))

            # Store map metadata
            self.map_info = {
                'resolution': map_data['resolution'],  # meters per pixel
                'origin': map_data['origin'],  # [x, y, theta]
                'width': self.map_image.shape[1],
                'height': self.map_image.shape[0],
            }

            self.get_logger().info(f'✓ Map loaded: {self.map_image.shape[1]}x{self.map_image.shape[0]} pixels')
            self.get_logger().info(f'  Resolution: {self.map_info["resolution"]:.4f} m/pixel')
            self.get_logger().info(f'  Origin: {self.map_info["origin"]}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error loading map: {e}')
            return False

    def world_to_map(self, x, y):
        """Convert world coordinates to map pixel coordinates."""
        if self.map_info is None:
            return x, y

        resolution = self.map_info['resolution']
        origin_x, origin_y = self.map_info['origin'][0], self.map_info['origin'][1]

        # Convert to pixel coordinates
        px = (x - origin_x) / resolution
        py = (y - origin_y) / resolution

        # Flip y-axis (image y increases downward, world y increases upward)
        py = self.map_info['height'] - py

        return px, py

    def load_global_path(self):
        """Load global path from CSV file."""
        if not self.global_path_csv:
            self.get_logger().error('No global path CSV file specified!')
            return False

        global_path_file = os.path.expanduser(self.global_path_csv)

        if not os.path.exists(global_path_file):
            self.get_logger().error(f'Global path file not found: {global_path_file}')
            return False

        try:
            # Read CSV file - expecting format: x,y,v,kappa (or similar)
            df = pd.read_csv(global_path_file)

            # Check for required columns
            if 'x' in df.columns and 'y' in df.columns:
                x_col, y_col = 'x', 'y'
            elif 'x_m' in df.columns and 'y_m' in df.columns:
                x_col, y_col = 'x_m', 'y_m'
            else:
                # Try to infer from first two columns
                x_col, y_col = df.columns[0], df.columns[1]
                self.get_logger().warn(f'Using columns "{x_col}" and "{y_col}" as x,y coordinates')

            self.global_path = {
                'x': df[x_col].values,
                'y': df[y_col].values,
            }

            self.get_logger().info(f'✓ Global path loaded: {len(self.global_path["x"])} waypoints')
            self.get_logger().info(f'  File: {global_path_file}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error loading global path: {e}')
            return False

    def run_analysis(self):
        """Run the complete analysis."""
        # Load map (optional)
        self.load_map()

        # Load global path
        if not self.load_global_path():
            self.get_logger().error('Failed to load global path!')
            rclpy.shutdown()
            return

        # Find CSV files to analyze
        csv_files = self.find_csv_files()

        if not csv_files:
            self.get_logger().error('No recording CSV files found!')
            rclpy.shutdown()
            return

        # Analyze each CSV file
        for csv_file in csv_files:
            self.analyze_trajectory(csv_file)

        self.get_logger().info('='*80)
        self.get_logger().info('Analysis complete!')
        if self.save_plots:
            self.get_logger().info(f'Plots saved to: {self.output_directory}')
        self.get_logger().info('='*80)

        # Shutdown after analysis
        rclpy.shutdown()

    def find_csv_files(self):
        """Find CSV files to analyze."""
        if self.csv_file:
            # Specific file provided
            csv_path = os.path.expanduser(self.csv_file)
            if os.path.exists(csv_path):
                self.get_logger().info(f'✓ Analyzing specific file: {csv_path}')
                return [csv_path]
            else:
                self.get_logger().error(f'CSV file not found: {csv_path}')
                return []
        else:
            # Find all CSV files in directory
            pattern = os.path.join(self.csv_directory, 'path_recording_*.csv')
            csv_files = sorted(glob.glob(pattern))

            if csv_files:
                self.get_logger().info(f'✓ Found {len(csv_files)} CSV files in {self.csv_directory}')
                # Analyze the most recent one by default
                self.get_logger().info(f'  Using most recent: {os.path.basename(csv_files[-1])}')
                return [csv_files[-1]]
            else:
                self.get_logger().error(f'No CSV files found in {self.csv_directory}')
                return []

    def analyze_trajectory(self, csv_file):
        """Analyze a single trajectory CSV file."""
        self.get_logger().info(f'\n✓ Analyzing: {os.path.basename(csv_file)}')

        try:
            # Read CSV file
            df = pd.read_csv(csv_file)

            self.get_logger().info(f'  Loaded {len(df)} data points')

            # Extract data
            recorded = {
                'x': df['x_m'].values,
                'y': df['y_m'].values,
                'vx': df['vx_mps'].values,
                'vy': df['vy_mps'].values,
                's': df['s_m'].values,
                'd': df['d_m (lateral_error)'].values,
                'timestamp': df['timestamp'].values,
            }

            # Calculate statistics
            stats = self.calculate_statistics(recorded)
            self.print_statistics(stats)

            # Generate plots
            self.generate_plots(recorded, csv_file, stats)

        except Exception as e:
            self.get_logger().error(f'Error analyzing {csv_file}: {e}')

    def calculate_statistics(self, recorded):
        """Calculate statistics for the trajectory."""
        lateral_errors = recorded['d']
        speeds = np.sqrt(recorded['vx']**2 + recorded['vy']**2)

        # Time statistics
        duration = recorded['timestamp'][-1] - recorded['timestamp'][0]

        # Distance traveled
        dx = np.diff(recorded['x'])
        dy = np.diff(recorded['y'])
        distances = np.sqrt(dx**2 + dy**2)
        total_distance = np.sum(distances)

        stats = {
            # Lateral error statistics
            'lat_error_mean': np.mean(lateral_errors),
            'lat_error_std': np.std(lateral_errors),
            'lat_error_min': np.min(lateral_errors),
            'lat_error_max': np.max(lateral_errors),
            'lat_error_abs_mean': np.mean(np.abs(lateral_errors)),
            'lat_error_abs_max': np.max(np.abs(lateral_errors)),
            'lat_error_rms': np.sqrt(np.mean(lateral_errors**2)),

            # Speed statistics
            'speed_mean': np.mean(speeds),
            'speed_std': np.std(speeds),
            'speed_min': np.min(speeds),
            'speed_max': np.max(speeds),

            # Time and distance
            'duration': duration,
            'total_distance': total_distance,
            'avg_speed': total_distance / duration if duration > 0 else 0,

            # Data points
            'num_points': len(recorded['x']),
        }

        return stats

    def print_statistics(self, stats):
        """Print statistics to console."""
        self.get_logger().info('')
        self.get_logger().info('  Statistics Summary:')
        self.get_logger().info('  ' + '-'*60)
        self.get_logger().info('  Lateral Error:')
        self.get_logger().info(f'    Mean:           {stats["lat_error_mean"]*100:7.2f} cm')
        self.get_logger().info(f'    Std Dev:        {stats["lat_error_std"]*100:7.2f} cm')
        self.get_logger().info(f'    RMS:            {stats["lat_error_rms"]*100:7.2f} cm')
        self.get_logger().info(f'    Abs Mean:       {stats["lat_error_abs_mean"]*100:7.2f} cm')
        self.get_logger().info(f'    Abs Max:        {stats["lat_error_abs_max"]*100:7.2f} cm')
        self.get_logger().info(f'    Range:          [{stats["lat_error_min"]*100:7.2f}, {stats["lat_error_max"]*100:7.2f}] cm')
        self.get_logger().info('')
        self.get_logger().info('  Speed:')
        self.get_logger().info(f'    Mean:           {stats["speed_mean"]:7.2f} m/s')
        self.get_logger().info(f'    Std Dev:        {stats["speed_std"]:7.2f} m/s')
        self.get_logger().info(f'    Range:          [{stats["speed_min"]:7.2f}, {stats["speed_max"]:7.2f}] m/s')
        self.get_logger().info('')
        self.get_logger().info('  Performance:')
        self.get_logger().info(f'    Duration:       {stats["duration"]:7.2f} s')
        self.get_logger().info(f'    Distance:       {stats["total_distance"]:7.2f} m')
        self.get_logger().info(f'    Avg Speed:      {stats["avg_speed"]:7.2f} m/s')
        self.get_logger().info(f'    Data Points:    {stats["num_points"]:7d}')
        self.get_logger().info('  ' + '-'*60)

    def generate_plots(self, recorded, csv_file, stats):
        """Generate comprehensive matplotlib visualizations."""
        # Create figure with subplots
        fig = plt.figure(figsize=(16, 12))
        gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)

        # Extract filename for title
        filename = os.path.basename(csv_file)
        fig.suptitle(f'Path Recording Analysis: {filename}', fontsize=16, fontweight='bold')

        # 1. Full trajectory comparison (top left, spans 2 columns)
        ax1 = fig.add_subplot(gs[0, :2])
        self.plot_trajectory_comparison(ax1, recorded)

        # 2. Zoomed trajectory comparison (top right)
        ax2 = fig.add_subplot(gs[0, 2])
        self.plot_trajectory_zoom(ax2, recorded)

        # 3. Lateral error over distance (middle left)
        ax3 = fig.add_subplot(gs[1, 0])
        self.plot_lateral_error_vs_distance(ax3, recorded)

        # 4. Lateral error over time (middle center)
        ax4 = fig.add_subplot(gs[1, 1])
        self.plot_lateral_error_vs_time(ax4, recorded)

        # 5. Lateral error histogram (middle right)
        ax5 = fig.add_subplot(gs[1, 2])
        self.plot_lateral_error_histogram(ax5, recorded)

        # 6. Speed profile (bottom left)
        ax6 = fig.add_subplot(gs[2, 0])
        self.plot_speed_profile(ax6, recorded)

        # 7. Speed vs lateral error (bottom center)
        ax7 = fig.add_subplot(gs[2, 1])
        self.plot_speed_vs_error(ax7, recorded)

        # 8. Statistics summary (bottom right)
        ax8 = fig.add_subplot(gs[2, 2])
        self.plot_statistics_summary(ax8, stats)

        # Save and/or show
        if self.save_plots:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_file = os.path.join(self.output_directory, f'analysis_{timestamp}.png')
            plt.savefig(output_file, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'  ✓ Plot saved: {output_file}')

        if self.show_plots:
            plt.show()
        else:
            plt.close()

    def plot_trajectory_comparison(self, ax, recorded):
        """Plot full trajectory comparison with map overlay."""
        # If map is loaded, show it as background
        if self.map_image is not None and self.map_info is not None:
            # Display map image
            ax.imshow(self.map_image, cmap='gray', origin='upper',
                     extent=[0, self.map_info['width'], 0, self.map_info['height']])

            # Convert world coordinates to map coordinates
            global_px, global_py = self.world_to_map(self.global_path['x'], self.global_path['y'])
            recorded_px, recorded_py = self.world_to_map(recorded['x'], recorded['y'])

            # Plot on map coordinates
            ax.plot(global_px, global_py, 'g-', linewidth=3, label='Global Centerline', alpha=0.8)

            speeds = np.sqrt(recorded['vx']**2 + recorded['vy']**2)
            scatter = ax.scatter(recorded_px, recorded_py,
                                c=speeds, cmap='jet', s=20,
                                label='Recorded Path', alpha=0.9, edgecolors='black', linewidths=0.5)

            ax.set_xlabel('Map X [pixels]', fontsize=10)
            ax.set_ylabel('Map Y [pixels]', fontsize=10)
            ax.set_title('Trajectory on Map (colored by speed)', fontsize=11, fontweight='bold')

        else:
            # No map - plot in world coordinates
            ax.plot(self.global_path['x'], self.global_path['y'],
                    'g-', linewidth=2, label='Global Centerline', alpha=0.7)

            speeds = np.sqrt(recorded['vx']**2 + recorded['vy']**2)
            scatter = ax.scatter(recorded['x'], recorded['y'],
                                c=speeds, cmap='jet', s=10,
                                label='Recorded Path', alpha=0.8)

            ax.set_xlabel('X [m]', fontsize=10)
            ax.set_ylabel('Y [m]', fontsize=10)
            ax.set_title('Trajectory Comparison (colored by speed)', fontsize=11, fontweight='bold')
            ax.axis('equal')

        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)

        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Speed [m/s]', fontsize=9)

    def plot_trajectory_zoom(self, ax, recorded):
        """Plot zoomed trajectory comparison."""
        # Take first 100 points for zoom
        n_zoom = min(100, len(recorded['x']))

        if self.map_image is not None and self.map_info is not None:
            # With map background
            ax.imshow(self.map_image, cmap='gray', origin='upper',
                     extent=[0, self.map_info['width'], 0, self.map_info['height']])

            global_px, global_py = self.world_to_map(self.global_path['x'], self.global_path['y'])
            recorded_px, recorded_py = self.world_to_map(recorded['x'][:n_zoom], recorded['y'][:n_zoom])
            start_px, start_py = self.world_to_map(np.array([recorded['x'][0]]), np.array([recorded['y'][0]]))

            ax.plot(global_px, global_py, 'g-', linewidth=2, label='Global Centerline', alpha=0.7)
            ax.plot(recorded_px, recorded_py, 'b-', linewidth=2, label='Recorded Path', alpha=0.8)
            ax.plot(start_px[0], start_py[0], 'ro', markersize=10, label='Start', markeredgecolor='white', markeredgewidth=2)

            # Zoom to start area (in pixel coordinates)
            margin = 100  # pixels
            ax.set_xlim(start_px[0] - margin, start_px[0] + margin)
            ax.set_ylim(start_py[0] - margin, start_py[0] + margin)

            ax.set_xlabel('Map X [pixels]', fontsize=10)
            ax.set_ylabel('Map Y [pixels]', fontsize=10)

        else:
            # Without map background
            ax.plot(self.global_path['x'], self.global_path['y'],
                    'g-', linewidth=2, label='Global Centerline', alpha=0.7)
            ax.plot(recorded['x'][:n_zoom], recorded['y'][:n_zoom],
                    'b-', linewidth=1.5, label='Recorded Path', alpha=0.8)
            ax.plot(recorded['x'][0], recorded['y'][0],
                    'ro', markersize=8, label='Start')

            ax.set_xlabel('X [m]', fontsize=10)
            ax.set_ylabel('Y [m]', fontsize=10)
            ax.axis('equal')

        ax.set_title('Trajectory (Zoomed Start)', fontsize=11, fontweight='bold')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)

    def plot_lateral_error_vs_distance(self, ax, recorded):
        """Plot lateral error along the path."""
        ax.plot(recorded['s'], recorded['d'] * 100, 'b-', linewidth=1, alpha=0.7)
        ax.axhline(y=0, color='g', linestyle='--', linewidth=1, label='Centerline')
        ax.fill_between(recorded['s'], recorded['d'] * 100, 0, alpha=0.3)

        ax.set_xlabel('Distance along path [m]', fontsize=10)
        ax.set_ylabel('Lateral Error [cm]', fontsize=10)
        ax.set_title('Lateral Error vs Distance', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)

    def plot_lateral_error_vs_time(self, ax, recorded):
        """Plot lateral error over time."""
        time_relative = recorded['timestamp'] - recorded['timestamp'][0]

        ax.plot(time_relative, recorded['d'] * 100, 'r-', linewidth=1, alpha=0.7)
        ax.axhline(y=0, color='g', linestyle='--', linewidth=1)
        ax.fill_between(time_relative, recorded['d'] * 100, 0, alpha=0.3, color='red')

        ax.set_xlabel('Time [s]', fontsize=10)
        ax.set_ylabel('Lateral Error [cm]', fontsize=10)
        ax.set_title('Lateral Error vs Time', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)

    def plot_lateral_error_histogram(self, ax, recorded):
        """Plot histogram of lateral errors."""
        errors_cm = recorded['d'] * 100

        ax.hist(errors_cm, bins=50, color='blue', alpha=0.7, edgecolor='black')
        ax.axvline(x=0, color='g', linestyle='--', linewidth=2, label='Centerline')
        ax.axvline(x=np.mean(errors_cm), color='r', linestyle='--',
                   linewidth=2, label=f'Mean: {np.mean(errors_cm):.2f} cm')

        ax.set_xlabel('Lateral Error [cm]', fontsize=10)
        ax.set_ylabel('Count', fontsize=10)
        ax.set_title('Lateral Error Distribution', fontsize=11, fontweight='bold')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)

    def plot_speed_profile(self, ax, recorded):
        """Plot speed profile."""
        speeds = np.sqrt(recorded['vx']**2 + recorded['vy']**2)
        time_relative = recorded['timestamp'] - recorded['timestamp'][0]

        ax.plot(time_relative, speeds, 'b-', linewidth=1.5, alpha=0.7)
        ax.fill_between(time_relative, speeds, 0, alpha=0.3)
        ax.axhline(y=np.mean(speeds), color='r', linestyle='--',
                   linewidth=1.5, label=f'Mean: {np.mean(speeds):.2f} m/s')

        ax.set_xlabel('Time [s]', fontsize=10)
        ax.set_ylabel('Speed [m/s]', fontsize=10)
        ax.set_title('Speed Profile', fontsize=11, fontweight='bold')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)

    def plot_speed_vs_error(self, ax, recorded):
        """Plot speed vs lateral error correlation."""
        speeds = np.sqrt(recorded['vx']**2 + recorded['vy']**2)
        errors_cm = np.abs(recorded['d']) * 100

        ax.scatter(speeds, errors_cm, alpha=0.5, s=10)

        # Add trend line
        z = np.polyfit(speeds, errors_cm, 1)
        p = np.poly1d(z)
        ax.plot(speeds, p(speeds), "r--", linewidth=2,
                label=f'Trend: y={z[0]:.2f}x+{z[1]:.2f}')

        ax.set_xlabel('Speed [m/s]', fontsize=10)
        ax.set_ylabel('Abs Lateral Error [cm]', fontsize=10)
        ax.set_title('Speed vs Lateral Error', fontsize=11, fontweight='bold')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)

    def plot_statistics_summary(self, ax, stats):
        """Plot statistics summary as text."""
        ax.axis('off')

        summary_text = f"""
STATISTICS SUMMARY

Lateral Error:
  Mean:        {stats['lat_error_mean']*100:7.2f} cm
  Std Dev:     {stats['lat_error_std']*100:7.2f} cm
  RMS:         {stats['lat_error_rms']*100:7.2f} cm
  Abs Mean:    {stats['lat_error_abs_mean']*100:7.2f} cm
  Abs Max:     {stats['lat_error_abs_max']*100:7.2f} cm

Speed:
  Mean:        {stats['speed_mean']:7.2f} m/s
  Std Dev:     {stats['speed_std']:7.2f} m/s
  Max:         {stats['speed_max']:7.2f} m/s

Performance:
  Duration:    {stats['duration']:7.2f} s
  Distance:    {stats['total_distance']:7.2f} m
  Avg Speed:   {stats['avg_speed']:7.2f} m/s
  Points:      {stats['num_points']:7d}
"""

        ax.text(0.1, 0.5, summary_text, fontsize=10,
                verticalalignment='center', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
        ax.set_title('Summary', fontsize=11, fontweight='bold')


def main(args=None):
    """Main entry point for offline analysis."""
    rclpy.init(args=args)

    node = OfflineAnalysisNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
