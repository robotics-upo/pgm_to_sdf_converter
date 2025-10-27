"""
Wall Aggregation Module - Classical Two-Pass Approach

Aggregates occupied pixels into horizontal and vertical rectangular segments.

Strategy:
1. Pass 1: Scan row-by-row to find maximal horizontal segments
2. Pass 2: Scan column-by-column to find maximal vertical segments (on remaining pixels)
3. Merge adjacent aligned segments to reduce segment count

This creates an optimized representation of occupied space as rectangles,
avoiding the need to create individual Gazebo objects for each pixel.
"""

import numpy as np
from typing import List, Tuple
from dataclasses import dataclass
from .segment_merger import SegmentMerger


@dataclass
class Wall:
    """Represents a wall segment in world coordinates."""

    x: float
    y: float
    width: float
    length: float
    angle: float = 0.0


class WallAggregationExtractor:
    """
    Extract walls using classical horizontal/vertical aggregation.

    This is the most efficient representation - groups adjacent occupied pixels
    into the fewest possible rectangles.
    """

    def __init__(
        self,
        image_data: np.ndarray,
        resolution: float,
        origin: Tuple[float, float],
        occupied_thresh: float = 0.65,
        free_thresh: float = 0.25,
        negate: int = 0,
        wall_height: float = 2.5,
        enable_merging: bool = True,
        merge_tolerance: float = 0.05,
    ):
        """
        Initialize the wall aggregation extractor.

        Args:
            image_data: Binary or grayscale image data
            resolution: Map resolution in meters/pixel
            origin: Map origin (x, y) in world coordinates
            occupied_thresh: Threshold for occupied pixels (default: 0.65)
            free_thresh: Threshold for free pixels (default: 0.25)
            negate: Whether to negate the image (default: 0)
            wall_height: Height of walls in meters (default: 2.5)
            enable_merging: Enable segment merging to combine adjacent aligned segments (default: True)
            merge_tolerance: Maximum gap between segments to merge in meters (default: 0.05m = 5cm)
        """
        self.image_data = image_data
        self.resolution = resolution
        self.origin = origin
        self.occupied_thresh = occupied_thresh
        self.free_thresh = free_thresh
        self.negate = negate
        self.wall_height = wall_height
        self.enable_merging = enable_merging

        self.height, self.width = image_data.shape

        if enable_merging:
            self.merger = SegmentMerger(merge_tolerance=merge_tolerance)
        else:
            self.merger = None

    def extract_walls(self, **kwargs) -> List[Wall]:
        """
        Extract walls using two-pass aggregation.

        Aggregates all occupied pixels into wall segments using:
        1. Horizontal scan (row-by-row)
        2. Vertical scan (column-by-column on remaining pixels)

        Returns:
            List of Wall objects
        """
        print("─" * 70)
        print("  WALL EXTRACTION")
        print("─" * 70)

        occupied_grid = self._get_occupied_grid()
        h_segments = self._aggregate_horizontal(occupied_grid)
        v_segments = self._aggregate_vertical(occupied_grid, h_segments)

        all_segments = h_segments + v_segments
        total_before_merge = len(all_segments)

        if self.enable_merging and self.merger is not None:
            all_segments = self.merger.merge_segments(all_segments, verbose=False)

        walls = self._segments_to_walls(all_segments)

        print(f"\n  Wall segments: {len(walls)}")
        if self.enable_merging and total_before_merge > len(walls):
            reduction_pct = (total_before_merge - len(walls)) / total_before_merge * 100
            print(
                f"  Merged from:   {total_before_merge} ({reduction_pct:.0f}% reduction)"
            )
        print()

        return walls

    def _get_occupied_grid(self) -> np.ndarray:
        """Create binary occupancy grid from image data."""
        if self.image_data.max() > 1:
            normalized = self.image_data.astype(np.float32) / 255.0
        else:
            normalized = self.image_data.astype(np.float32)

        if self.negate:
            occupancy_prob = 1.0 - normalized
        else:
            occupancy_prob = normalized

        occupied = (occupancy_prob >= self.occupied_thresh).astype(np.uint8)
        return occupied

    def _aggregate_horizontal(self, grid: np.ndarray) -> List[dict]:
        """Find all maximal horizontal segments."""
        height, width = grid.shape
        horizontal_segments = []

        for y in range(height):
            x = 0
            while x < width:
                if grid[y, x] == 1 and (x == 0 or grid[y, x - 1] == 0):
                    start_x = x

                    end_x = x
                    while end_x + 1 < width and grid[y, end_x + 1] == 1:
                        end_x += 1

                    segment = {
                        "type": "horizontal",
                        "y": y,
                        "start_x": start_x,
                        "end_x": end_x,
                        "length_px": end_x - start_x + 1,
                    }
                    horizontal_segments.append(segment)

                    x = end_x + 1
                else:
                    x += 1

        return horizontal_segments

    def _aggregate_vertical(
        self, grid: np.ndarray, horizontal_segments: List[dict]
    ) -> List[dict]:
        """Find all maximal vertical segments from remaining pixels."""
        height, width = grid.shape
        grid_for_vertical = grid.copy()

        for seg in horizontal_segments:
            if seg["length_px"] > 1:
                y = seg["y"]
                start_x = seg["start_x"]
                end_x = seg["end_x"]
                grid_for_vertical[y, start_x : end_x + 1] = 0

        vertical_segments = []

        for x in range(width):
            y = 0
            while y < height:
                if grid_for_vertical[y, x] == 1 and (
                    y == 0 or grid_for_vertical[y - 1, x] == 0
                ):
                    start_y = y

                    end_y = y
                    while end_y + 1 < height and grid_for_vertical[end_y + 1, x] == 1:
                        end_y += 1

                    segment = {
                        "type": "vertical",
                        "x": x,
                        "start_y": start_y,
                        "end_y": end_y,
                        "length_px": end_y - start_y + 1,
                    }
                    vertical_segments.append(segment)

                    y = end_y + 1
                else:
                    y += 1

        return vertical_segments

    def _segments_to_walls(self, segments: List[dict]) -> List[Wall]:
        """Convert segments to Wall objects."""
        walls = []

        for seg in segments:
            if seg["type"] == "horizontal":
                start_x_px = seg["start_x"]
                end_x_px = seg["end_x"]

                if "start_y" in seg and "end_y" in seg:
                    start_y_px = seg["start_y"]
                    end_y_px = seg["end_y"]
                else:
                    start_y_px = seg["y"]
                    end_y_px = seg["y"]

                center_x_px = (start_x_px + end_x_px) / 2.0
                center_y_px = (start_y_px + end_y_px) / 2.0

                width_px = end_x_px - start_x_px + 1
                height_px = end_y_px - start_y_px + 1

            else:
                start_y_px = seg["start_y"]
                end_y_px = seg["end_y"]

                if "start_x" in seg and "end_x" in seg:
                    start_x_px = seg["start_x"]
                    end_x_px = seg["end_x"]
                else:
                    start_x_px = seg["x"]
                    end_x_px = seg["x"]

                center_x_px = (start_x_px + end_x_px) / 2.0
                center_y_px = (start_y_px + end_y_px) / 2.0

                width_px = end_x_px - start_x_px + 1
                height_px = end_y_px - start_y_px + 1

            center_world = self._pixel_to_world(np.array([center_x_px, center_y_px]))
            width_m = width_px * self.resolution
            length_m = height_px * self.resolution

            wall = Wall(
                x=center_world[0],
                y=center_world[1],
                width=width_m,
                length=length_m,
                angle=0.0,
            )
            walls.append(wall)

        return walls

    def _pixel_to_world(self, pixel_coords: np.ndarray) -> Tuple[float, float]:
        """Convert pixel coordinates to world coordinates."""
        x_px, y_px = pixel_coords
        x_world = self.origin[0] + x_px * self.resolution
        y_world = self.origin[1] + (self.height - y_px) * self.resolution
        return (x_world, y_world)
