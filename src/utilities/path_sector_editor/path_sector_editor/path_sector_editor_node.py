#!/usr/bin/env python3
"""
Interactive Sector Tuner Node with Matplotlib GUI
Allows sector-based path tuning with real-time visualization
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider, TextBox, CheckButtons
import matplotlib.patches as mpatches
from matplotlib.colors import Normalize
import matplotlib.cm as cm
import numpy as np
from path_sector_editor.sector_manager import SectorManager, Sector
from typing import Optional, List
from PIL import Image
import os
import yaml


class SectorTunerNode(Node):
    """ROS2 node for interactive sector-based path tuning"""

    def __init__(self):
        super().__init__('path_sector_editor_node')

        # Parameters
        self.declare_parameter('csv_file_path', '')
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('smooth_transition_points', 20)
        self.declare_parameter('global_path_topic', '/global_centerline')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)

        # Get parameters
        self.csv_path = self.get_parameter('csv_file_path').value
        self.map_yaml_path = self.get_parameter('map_yaml_path').value
        smooth_points = self.get_parameter('smooth_transition_points').value
        path_topic = self.get_parameter('global_path_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        pub_rate = self.get_parameter('publish_rate').value

        # Map metadata (will be loaded from yaml)
        self.map_image_path = None
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # Publisher
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / pub_rate, self.publish_path)

        # Sector manager with smoothing parameter
        self.manager = SectorManager(smooth_transition_points=smooth_points)

        # GUI state
        self.selecting_sector = False
        self.sector_start_idx = None
        self.sector_end_idx = None
        self.selected_sector: Optional[Sector] = None
        self.status_text = None
        self.map_image = None
        self.map_extent = None
        self.colorbar = None  # Store colorbar reference to prevent duplicates
        self.show_colorbar = True  # Toggle for velocity colorbar

        # Initialize
        self.get_logger().info(f"Path Sector Editor Node started")
        self.get_logger().info(f"Publishing to: {path_topic}")
        self.get_logger().info(f"Smooth transition points: {smooth_points}")

        # Load map metadata from yaml and then load image
        if self.map_yaml_path:
            self.load_map_from_yaml()
        else:
            self.get_logger().warn("No map_yaml_path specified, map will not be displayed")

        # Load CSV if provided
        if self.csv_path:
            if self.manager.load_csv(self.csv_path):
                self.get_logger().info(f"Loaded path from: {self.csv_path}")
            else:
                self.get_logger().error(f"Failed to load: {self.csv_path}")

    def load_map_from_yaml(self):
        """Load map metadata from YAML file and then load the image"""
        if not os.path.exists(self.map_yaml_path):
            self.get_logger().error(f"Map YAML file not found: {self.map_yaml_path}")
            return

        try:
            # Load YAML file
            with open(self.map_yaml_path, 'r') as f:
                map_config = yaml.safe_load(f)

            # Extract metadata
            self.map_image_path = map_config.get('image', '')
            self.map_resolution = map_config.get('resolution', 0.05)
            origin = map_config.get('origin', [0.0, 0.0, 0.0])
            self.map_origin_x = origin[0]
            self.map_origin_y = origin[1]

            self.get_logger().info(f"Loaded map config from: {self.map_yaml_path}")
            self.get_logger().info(f"  Image: {self.map_image_path}")
            self.get_logger().info(f"  Resolution: {self.map_resolution} m/pixel")
            self.get_logger().info(f"  Origin: ({self.map_origin_x}, {self.map_origin_y})")

            # Handle relative paths
            if not os.path.isabs(self.map_image_path):
                # If relative, make it relative to yaml file directory
                yaml_dir = os.path.dirname(self.map_yaml_path)
                self.map_image_path = os.path.join(yaml_dir, self.map_image_path)

            # Now load the image
            self.load_map_image()

        except Exception as e:
            self.get_logger().error(f"Failed to load map YAML: {e}")
            self.map_image = None

    def load_map_image(self):
        """Load map image file (pgm or png)"""
        if not self.map_image_path or not os.path.exists(self.map_image_path):
            self.get_logger().warn(f"Map image not found: {self.map_image_path}")
            return

        try:
            # Load image
            img = Image.open(self.map_image_path)

            # Convert to grayscale if needed
            if img.mode != 'L':
                img = img.convert('L')

            # Convert to numpy array and flip vertically (image origin is top-left, map origin is bottom-left)
            self.map_image = np.flipud(np.array(img))

            # Calculate map extent in world coordinates
            height, width = self.map_image.shape
            self.map_extent = [
                self.map_origin_x,
                self.map_origin_x + width * self.map_resolution,
                self.map_origin_y,
                self.map_origin_y + height * self.map_resolution
            ]

            self.get_logger().info(f"Loaded map image: {width}x{height} pixels")
            self.get_logger().info(f"Map extent: {self.map_extent}")

        except Exception as e:
            self.get_logger().error(f"Failed to load map image: {e}")
            self.map_image = None

    def publish_path(self):
        """Publish modified path to ROS topic"""
        if self.manager.modified_waypoints is None:
            return

        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for waypoint in self.manager.modified_waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def run_gui(self):
        """Run interactive matplotlib GUI"""
        if self.manager.waypoints is None:
            self.get_logger().error("No waypoints loaded. Please specify csv_file_path parameter.")
            return

        # Create figure with subplots
        self.fig = plt.figure(figsize=(18, 10))
        self.fig.canvas.manager.set_window_title('Path Sector Editor - F1TENTH')

        # Main plot for track
        self.ax_track = plt.subplot2grid((3, 3), (0, 0), colspan=2, rowspan=3)
        self.ax_track.set_title('Track Map (Click to define sectors)', fontsize=12, fontweight='bold')
        self.ax_track.set_xlabel('X (m)')
        self.ax_track.set_ylabel('Y (m)')
        self.ax_track.set_aspect('equal')
        self.ax_track.grid(True, alpha=0.3)

        # Status text
        self.status_text = self.ax_track.text(0.02, 0.98, '', transform=self.ax_track.transAxes,
                                            fontsize=10, verticalalignment='top',
                                            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))

        # Control panel area
        control_x = 0.68
        control_y_start = 0.88
        control_spacing = 0.06

        # Instructions
        instructions = (
            "SECTOR DEFINITION:\n"
            "1. Click 'New Sector'\n"
            "2. Click start point\n"
            "3. Click end point\n"
            "   (Name auto-generated)\n\n"
            "SECTOR TUNING:\n"
            "1. Click sector to select\n"
            "2. Adjust sliders\n"
            "3. Click 'Apply'\n\n"
            "SMOOTHING:\n"
            "• Transition Points:\n"
            "  smooth connection\n"
            "  at sector borders"
        )
        self.ax_instructions = plt.subplot2grid((3, 3), (0, 2), rowspan=1)
        self.ax_instructions.axis('off')
        self.ax_instructions.text(0.05, 0.95, instructions, transform=self.ax_instructions.transAxes,
                                 fontsize=8, verticalalignment='top', family='monospace',
                                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

        # Sector list area
        self.ax_sector_list = plt.subplot2grid((3, 3), (1, 2), rowspan=2)
        self.ax_sector_list.axis('off')

        # Sliders
        slider_width = 0.25
        slider_height = 0.025

        # Velocity scale slider
        ax_vel_scale = plt.axes([control_x, control_y_start - control_spacing * 0, slider_width, slider_height])
        self.slider_vel_scale = Slider(ax_vel_scale, 'Vel Scale', 0.1, 2.0, valinit=1.0, valstep=0.05)

        # Velocity offset slider
        ax_vel_offset = plt.axes([control_x, control_y_start - control_spacing * 1, slider_width, slider_height])
        self.slider_vel_offset = Slider(ax_vel_offset, 'Vel Offset', -2.0, 2.0, valinit=0.0, valstep=0.1)

        # D offset slider
        ax_d_offset = plt.axes([control_x, control_y_start - control_spacing * 2, slider_width, slider_height])
        self.slider_d_offset = Slider(ax_d_offset, 'D Offset (m)', -1.0, 1.0, valinit=0.0, valstep=0.05)

        # Smooth transition points slider
        ax_smooth = plt.axes([control_x, control_y_start - control_spacing * 3, slider_width, slider_height])
        self.slider_smooth = Slider(ax_smooth, 'Smooth Pts', 0, 50,
                                     valinit=self.manager.smooth_transition_points, valstep=1)
        self.slider_smooth.on_changed(self.on_smooth_changed)

        # Buttons
        button_width = 0.12
        button_height = 0.035
        button_x1 = control_x
        button_x2 = control_x + button_width + 0.01
        button_y = control_y_start - control_spacing * 4.5

        # New sector button
        ax_new_sector = plt.axes([button_x1, button_y, button_width, button_height])
        self.btn_new_sector = Button(ax_new_sector, 'New Sector', color='lightgreen')
        self.btn_new_sector.on_clicked(self.on_new_sector)

        # Delete sector button
        ax_del_sector = plt.axes([button_x2, button_y, button_width, button_height])
        self.btn_del_sector = Button(ax_del_sector, 'Delete', color='lightcoral')
        self.btn_del_sector.on_clicked(self.on_delete_sector)

        button_y -= (button_height + 0.01)

        # Apply button
        ax_apply = plt.axes([button_x1, button_y, button_width, button_height])
        self.btn_apply = Button(ax_apply, 'Apply', color='lightblue')
        self.btn_apply.on_clicked(self.on_apply)

        # Reset button
        ax_reset = plt.axes([button_x2, button_y, button_width, button_height])
        self.btn_reset = Button(ax_reset, 'Reset', color='lightyellow')
        self.btn_reset.on_clicked(self.on_reset)

        button_y -= (button_height + 0.01)

        # Save button
        ax_save = plt.axes([button_x1, button_y, button_width, button_height])
        self.btn_save = Button(ax_save, 'Save CSV', color='lightgreen')
        self.btn_save.on_clicked(self.on_save)

        # Clear sectors button
        ax_clear = plt.axes([button_x2, button_y, button_width, button_height])
        self.btn_clear = Button(ax_clear, 'Clear All', color='lightcoral')
        self.btn_clear.on_clicked(self.on_clear_sectors)

        button_y -= (button_height + 0.02)

        # Checkbox for colorbar toggle
        ax_checkbox = plt.axes([control_x, button_y, slider_width, button_height])
        self.checkbox_colorbar = CheckButtons(ax_checkbox, ['Show Velocity Bar'], [self.show_colorbar])
        self.checkbox_colorbar.on_clicked(self.on_colorbar_toggle)

        # Connect mouse click event
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        # Initial plot
        self.update_plot()

        # Show GUI
        plt.tight_layout()
        plt.show()

    def on_smooth_changed(self, val):
        """Handle smooth transition points slider change"""
        self.manager.smooth_transition_points = int(val)
        self.get_logger().info(f"Smooth transition points: {int(val)}")

    def on_colorbar_toggle(self, label):
        """Handle colorbar visibility toggle"""
        self.show_colorbar = not self.show_colorbar
        self.get_logger().info(f"Colorbar visibility: {self.show_colorbar}")
        self.update_plot()

    def update_status(self, message: str):
        """Update status message"""
        if self.status_text:
            self.status_text.set_text(message)
            self.fig.canvas.draw_idle()

    def update_plot(self):
        """Update track visualization with map background and velocity colormap"""
        # Store current axis limits to maintain zoom level
        xlim = self.ax_track.get_xlim() if hasattr(self, 'ax_track') else None
        ylim = self.ax_track.get_ylim() if hasattr(self, 'ax_track') else None

        self.ax_track.clear()
        self.ax_track.set_title('Track Map (Velocity colored path)', fontsize=12, fontweight='bold')
        self.ax_track.set_xlabel('X (m)')
        self.ax_track.set_ylabel('Y (m)')
        self.ax_track.set_aspect('equal')
        self.ax_track.grid(True, alpha=0.3)

        # Recreate status text after clearing
        self.status_text = self.ax_track.text(0.02, 0.98, '', transform=self.ax_track.transAxes,
                                            fontsize=10, verticalalignment='top',
                                            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))

        waypoints = self.manager.modified_waypoints
        if waypoints is None:
            return

        # Display map image as background
        if self.map_image is not None:
            self.ax_track.imshow(self.map_image, cmap='gray', extent=self.map_extent,
                               alpha=0.6, origin='lower', zorder=0)

        # Create velocity colormap
        velocities = waypoints[:, 2]
        v_min, v_max = np.min(velocities), np.max(velocities)
        norm = Normalize(vmin=v_min, vmax=v_max)
        cmap = cm.get_cmap('jet')  # Red=slow, Blue=fast

        # Plot base path with velocity colors
        for i in range(len(waypoints) - 1):
            color = cmap(norm(velocities[i]))
            self.ax_track.plot(waypoints[i:i+2, 0], waypoints[i:i+2, 1],
                             color=color, linewidth=3, alpha=0.6, zorder=1)

        # Remove old colorbar if exists
        if self.colorbar is not None:
            self.colorbar.remove()
            self.colorbar = None

        # Add horizontal colorbar for velocity (only if enabled)
        if self.show_colorbar:
            sm = cm.ScalarMappable(cmap=cmap, norm=norm)
            sm.set_array([])
            self.colorbar = plt.colorbar(sm, ax=self.ax_track,
                                         orientation='horizontal',
                                         pad=0.08,
                                         fraction=0.046,
                                         aspect=40)
            self.colorbar.set_label('Velocity (m/s)', fontsize=10)

        # Plot sectors with highlights (behind global path)
        for sector in self.manager.sectors:
            indices = self.manager.get_sector_indices(sector)
            sector_waypoints = waypoints[indices]

            # Highlight selected sector with thicker solid line behind the path
            if sector == self.selected_sector:
                self.ax_track.plot(sector_waypoints[:, 0], sector_waypoints[:, 1],
                                 color=sector.color, linewidth=6, alpha=0.8,
                                 linestyle='-', zorder=0.5, label=f'{sector.name} (selected)')

            # Mark start point
            start_pt = waypoints[sector.start_idx]
            self.ax_track.plot(start_pt[0], start_pt[1], 'o', color=sector.color,
                             markersize=10, markeredgecolor='black', markeredgewidth=2, zorder=3)

            # Mark end point
            end_pt = waypoints[sector.end_idx - 1] if sector.end_idx > 0 else start_pt
            self.ax_track.plot(end_pt[0], end_pt[1], 's', color=sector.color,
                             markersize=8, markeredgecolor='black', markeredgewidth=2, zorder=3)

        # Mark selection start if in progress
        if self.selecting_sector and self.sector_start_idx is not None:
            start_pt = waypoints[self.sector_start_idx]
            self.ax_track.plot(start_pt[0], start_pt[1], 'r*', markersize=20,
                             markeredgecolor='black', markeredgewidth=1,
                             label='Start', zorder=4)

        # Add legend if there are sectors
        if len(self.manager.sectors) > 0 or self.selecting_sector:
            self.ax_track.legend(loc='upper right', fontsize=8)

        # Restore axis limits to maintain zoom level
        if xlim is not None and ylim is not None:
            # Only restore if limits were previously set (not initial plot)
            # Check if limits are not default matplotlib auto-scaled limits
            if xlim != (0.0, 1.0) or ylim != (0.0, 1.0):
                self.ax_track.set_xlim(xlim)
                self.ax_track.set_ylim(ylim)

        # Update sector list
        self.update_sector_list()

        self.fig.canvas.draw_idle()

    def update_sector_list(self):
        """Update sector list display"""
        self.ax_sector_list.clear()
        self.ax_sector_list.axis('off')

        if len(self.manager.sectors) == 0:
            self.ax_sector_list.text(0.05, 0.95, 'No sectors defined\n\nClick "New Sector"\nto start',
                                   transform=self.ax_sector_list.transAxes,
                                   fontsize=10, verticalalignment='top',
                                   bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.3))
            return

        # Create sector list text
        text = "SECTORS:\n" + "="*35 + "\n"
        for i, sector in enumerate(self.manager.sectors):
            marker = "►" if sector == self.selected_sector else " "
            text += f"{marker} {sector.name}\n"
            text += f"   [{sector.start_idx}:{sector.end_idx}]\n"
            text += f"   V: ×{sector.velocity_scale:.2f} "
            text += f"{sector.velocity_offset:+.2f}m/s\n"
            text += f"   D: {sector.d_offset:+.3f}m\n"
            if i < len(self.manager.sectors) - 1:
                text += "\n"

        self.ax_sector_list.text(0.05, 0.95, text, transform=self.ax_sector_list.transAxes,
                               fontsize=8, verticalalignment='top', family='monospace',
                               bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))

    def on_click(self, event):
        """Handle mouse clicks on track"""
        if event.inaxes != self.ax_track:
            return

        if not self.selecting_sector:
            # Click on sector to select it
            clicked_idx = self.manager.find_nearest_waypoint(event.xdata, event.ydata)

            # Find which sector this point belongs to
            for sector in self.manager.sectors:
                if sector.start_idx <= clicked_idx < sector.end_idx:
                    self.selected_sector = sector
                    self.update_sliders_from_sector(sector)
                    self.update_status(f"Selected: {sector.name}")
                    self.update_plot()
                    self.get_logger().info(f"Selected sector: {sector.name}")
                    break
        else:
            # Defining new sector
            clicked_idx = self.manager.find_nearest_waypoint(event.xdata, event.ydata)

            if self.sector_start_idx is None:
                # First click - start point
                self.sector_start_idx = clicked_idx
                self.update_status(f"Start: idx {clicked_idx}. Click END point.")
                self.get_logger().info(f"Sector start: index {clicked_idx}")
                self.update_plot()
            else:
                # Second click - end point (auto-create with generated name)
                self.sector_end_idx = clicked_idx

                # Automatically create sector with generated name
                sector = self.manager.add_sector(None, self.sector_start_idx, self.sector_end_idx)
                self.selected_sector = sector

                # Reset state
                self.selecting_sector = False
                self.sector_start_idx = None
                self.sector_end_idx = None

                self.update_status(f"Created: {sector.name}")
                self.update_plot()
                self.get_logger().info(f"Created sector: {sector.name}")

    def on_new_sector(self, event):
        """Start new sector definition"""
        self.selecting_sector = True
        self.sector_start_idx = None
        self.sector_end_idx = None
        self.selected_sector = None
        self.update_status("Click START point")
        self.get_logger().info("Click start point for new sector")
        self.update_plot()

    def on_delete_sector(self, event):
        """Delete selected sector"""
        if self.selected_sector:
            sector_name = self.selected_sector.name
            self.manager.remove_sector(self.selected_sector)
            self.selected_sector = None
            self.update_status(f"Deleted: {sector_name}")
            self.update_plot()
        else:
            self.update_status("No sector selected")

    def on_apply(self, event):
        """Apply current slider values to selected sector"""
        if self.selected_sector:
            self.selected_sector.velocity_scale = self.slider_vel_scale.val
            self.selected_sector.velocity_offset = self.slider_vel_offset.val
            self.selected_sector.d_offset = self.slider_d_offset.val

            self.manager.apply_modifications()
            self.update_status(f"Applied: {self.selected_sector.name}")
            self.update_plot()
            self.get_logger().info(f"Applied changes to: {self.selected_sector.name}")
        else:
            self.update_status("No sector selected!")

    def on_reset(self, event):
        """Reset all modifications"""
        self.manager.modified_waypoints = self.manager.waypoints.copy()
        for sector in self.manager.sectors:
            sector.velocity_scale = 1.0
            sector.velocity_offset = 0.0
            sector.d_offset = 0.0
        self.update_sliders_from_sector(None)
        self.update_status("Reset all")
        self.update_plot()
        self.get_logger().info("Reset all modifications")

    def on_save(self, event):
        """Save modified path to CSV"""
        if self.manager.save_csv():
            self.update_status("Saved to CSV")
            self.get_logger().info("Saved modified path")
        else:
            self.update_status("Save failed")

    def on_clear_sectors(self, event):
        """Clear all sectors"""
        self.manager.clear_sectors()
        self.manager.sector_counter = 0  # Reset counter
        self.selected_sector = None
        self.manager.modified_waypoints = self.manager.waypoints.copy()
        self.update_status("Cleared all sectors")
        self.update_plot()

    def update_sliders_from_sector(self, sector: Optional[Sector]):
        """Update slider values from sector"""
        if sector:
            self.slider_vel_scale.set_val(sector.velocity_scale)
            self.slider_vel_offset.set_val(sector.velocity_offset)
            self.slider_d_offset.set_val(sector.d_offset)
        else:
            self.slider_vel_scale.set_val(1.0)
            self.slider_vel_offset.set_val(0.0)
            self.slider_d_offset.set_val(0.0)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    node = SectorTunerNode()

    # Run GUI in main thread (matplotlib requires this)
    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
