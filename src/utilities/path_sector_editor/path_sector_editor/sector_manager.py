#!/usr/bin/env python3
"""
Sector Manager for path tuning
Handles CSV loading/saving, sector definitions, and path modifications
"""

import numpy as np
import csv
import copy
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, field


@dataclass
class Sector:
    """Represents a sector of the path"""
    name: str
    start_idx: int
    end_idx: int
    velocity_scale: float = 1.0
    velocity_offset: float = 0.0
    d_offset: float = 0.0  # Lateral offset in Frenet frame (meters)
    color: str = 'blue'


@dataclass
class UndoState:
    """Snapshot of manager state for undo functionality"""
    sectors: List[Sector]
    modified_waypoints: np.ndarray
    sector_counter: int
    selected_sector_name: Optional[str] = None


class SectorManager:
    """Manages sectors and path modifications"""

    def __init__(self, smooth_transition_points: int = 20, max_undo_history: int = 50):
        self.waypoints = None  # (N, 4) array: [x, y, v, kappa]
        self.sectors: List[Sector] = []
        self.file_path: Optional[str] = None
        self.modified_waypoints = None
        self.smooth_transition_points = smooth_transition_points
        self.sector_counter = 0  # Auto-naming counter

        # Undo functionality
        self.undo_history: List[UndoState] = []
        self.max_undo_history = max_undo_history

    def load_csv(self, file_path: str) -> bool:
        """Load waypoints from CSV file (x, y, v, kappa)"""
        try:
            data = []
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                header = next(reader)  # Skip header
                for row in reader:
                    if len(row) >= 4:
                        data.append([float(x) for x in row[:4]])

            self.waypoints = np.array(data)
            self.modified_waypoints = self.waypoints.copy()
            self.file_path = file_path
            print(f"Loaded {len(self.waypoints)} waypoints from {file_path}")
            return True
        except Exception as e:
            print(f"Error loading CSV: {e}")
            return False

    def save_csv(self, file_path: Optional[str] = None) -> bool:
        """Save modified waypoints to CSV file"""
        if file_path is None:
            if self.file_path is None:
                print("No file path specified")
                return False
            # Create new filename with _modified suffix
            base = self.file_path.rsplit('.', 1)[0]
            file_path = f"{base}_modified.csv"

        try:
            with open(file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'v', 'kappa'])
                for row in self.modified_waypoints:
                    writer.writerow(row)

            print(f"Saved {len(self.modified_waypoints)} waypoints to {file_path}")
            return True
        except Exception as e:
            print(f"Error saving CSV: {e}")
            return False

    def add_sector(self, name: str = None, start_idx: int = 0, end_idx: int = 0, color: str = None) -> Sector:
        """Add a new sector with auto-generated name if not provided"""
        # Auto-generate name if not provided
        if name is None or name.strip() == '':
            self.sector_counter += 1
            name = f"Sector {self.sector_counter}"

        if color is None:
            colors = ['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'magenta', 'yellow']
            color = colors[len(self.sectors) % len(colors)]

        # Handle wrap-around for closed tracks
        if end_idx < start_idx:
            end_idx = len(self.waypoints)

        sector = Sector(name, start_idx, end_idx, color=color)
        self.sectors.append(sector)
        print(f"Added sector '{name}': idx {start_idx}-{end_idx}")
        return sector

    def remove_sector(self, sector: Sector):
        """Remove a sector"""
        if sector in self.sectors:
            self.sectors.remove(sector)
            print(f"Removed sector '{sector.name}'")

    def clear_sectors(self):
        """Clear all sectors"""
        self.sectors.clear()
        print("Cleared all sectors")

    def apply_modifications(self):
        """Apply all sector modifications to waypoints"""
        if self.waypoints is None:
            print("No waypoints loaded")
            return

        # Start with original waypoints
        self.modified_waypoints = self.waypoints.copy()

        # Apply each sector's modifications
        for sector in self.sectors:
            self._apply_sector(sector)

        print(f"Applied modifications from {len(self.sectors)} sectors")

    def _apply_sector(self, sector: Sector):
        """Apply a single sector's modifications"""
        start = sector.start_idx
        end = sector.end_idx

        # Velocity modification: v_new = v_old * scale + offset
        v_original = self.waypoints[start:end, 2]
        v_modified = v_original * sector.velocity_scale + sector.velocity_offset
        v_modified = np.maximum(v_modified, 0.1)  # Minimum velocity
        self.modified_waypoints[start:end, 2] = v_modified

        # Lateral offset (d_offset) modification
        if abs(sector.d_offset) > 0.001:
            self._apply_lateral_offset(start, end, sector.d_offset)

    def _apply_lateral_offset(self, start: int, end: int, d_offset: float):
        """
        Apply lateral offset to waypoints with smooth transitions
        d_offset > 0: shift right (in vehicle frame)
        d_offset < 0: shift left (in vehicle frame)
        """
        n_smooth = self.smooth_transition_points
        sector_length = end - start

        # Adjust smoothing if sector is too short
        n_smooth = min(n_smooth, sector_length // 4)

        # Calculate path tangent vectors and apply offset
        for i in range(start, end):
            # Use forward difference for tangent (except last point)
            if i < len(self.waypoints) - 1:
                dx = self.waypoints[i+1, 0] - self.waypoints[i, 0]
                dy = self.waypoints[i+1, 1] - self.waypoints[i, 1]
            else:
                # Use backward difference for last point
                dx = self.waypoints[i, 0] - self.waypoints[i-1, 0]
                dy = self.waypoints[i, 1] - self.waypoints[i-1, 1]

            # Normalize tangent
            length = np.sqrt(dx**2 + dy**2)
            if length > 0.001:
                dx /= length
                dy /= length

                # Perpendicular vector (right-hand side, 90° CCW rotation)
                perp_x = -dy
                perp_y = dx

                # Calculate smooth transition weight
                weight = 1.0
                rel_pos = i - start

                # Smooth transition at start
                if rel_pos < n_smooth:
                    # Cosine smoothing: 0 -> 1
                    t = rel_pos / n_smooth
                    weight = 0.5 * (1.0 - np.cos(np.pi * t))

                # Smooth transition at end
                elif rel_pos >= sector_length - n_smooth:
                    # Cosine smoothing: 1 -> 0
                    t = (sector_length - rel_pos - 1) / n_smooth
                    weight = 0.5 * (1.0 - np.cos(np.pi * t))

                # Apply weighted offset
                self.modified_waypoints[i, 0] += perp_x * d_offset * weight
                self.modified_waypoints[i, 1] += perp_y * d_offset * weight

    def get_waypoint_at_index(self, idx: int) -> np.ndarray:
        """Get waypoint at specific index"""
        if self.waypoints is None or idx < 0 or idx >= len(self.waypoints):
            return None
        return self.waypoints[idx]

    def find_nearest_waypoint(self, x: float, y: float) -> int:
        """Find index of nearest waypoint to given position"""
        if self.waypoints is None:
            return -1

        distances = np.sqrt((self.waypoints[:, 0] - x)**2 + (self.waypoints[:, 1] - y)**2)
        return int(np.argmin(distances))

    def get_sector_indices(self, sector: Sector) -> np.ndarray:
        """Get array of indices for a sector"""
        return np.arange(sector.start_idx, sector.end_idx)

    def get_path_bounds(self) -> Tuple[float, float, float, float]:
        """Get path bounding box (min_x, max_x, min_y, max_y)"""
        if self.waypoints is None:
            return (0, 1, 0, 1)

        return (
            np.min(self.waypoints[:, 0]),
            np.max(self.waypoints[:, 0]),
            np.min(self.waypoints[:, 1]),
            np.max(self.waypoints[:, 1])
        )

    def save_state(self, selected_sector: Optional[Sector] = None):
        """
        Save current state to undo history

        Args:
            selected_sector: Currently selected sector (optional)
        """
        if self.waypoints is None:
            return

        # Create deep copy of sectors
        sectors_copy = copy.deepcopy(self.sectors)

        # Copy waypoints
        waypoints_copy = self.modified_waypoints.copy() if self.modified_waypoints is not None else None

        # Get selected sector name
        selected_name = selected_sector.name if selected_sector else None

        # Create state snapshot
        state = UndoState(
            sectors=sectors_copy,
            modified_waypoints=waypoints_copy,
            sector_counter=self.sector_counter,
            selected_sector_name=selected_name
        )

        # Add to history
        self.undo_history.append(state)

        # Limit history size (FIFO)
        if len(self.undo_history) > self.max_undo_history:
            self.undo_history.pop(0)

        print(f"State saved (history size: {len(self.undo_history)})")

    def undo(self) -> Tuple[bool, Optional[str]]:
        """
        Restore previous state from undo history

        Returns:
            Tuple of (success, selected_sector_name)
        """
        if len(self.undo_history) == 0:
            print("No undo history available")
            return False, None

        # Pop last state
        state = self.undo_history.pop()

        # Restore state
        self.sectors = copy.deepcopy(state.sectors)
        self.modified_waypoints = state.modified_waypoints.copy() if state.modified_waypoints is not None else None
        self.sector_counter = state.sector_counter

        print(f"Undo successful (history size: {len(self.undo_history)})")
        return True, state.selected_sector_name

    def can_undo(self) -> bool:
        """Check if undo is available"""
        return len(self.undo_history) > 0

    def clear_undo_history(self):
        """Clear all undo history"""
        self.undo_history.clear()
        print("Undo history cleared")
