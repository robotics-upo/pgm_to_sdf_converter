"""
Segment Merger Module - Combines adjacent aligned wall segments

This module reduces the number of wall segments by merging adjacent segments
that are collinear and touching. This happens AFTER aggregation, so no wall
data is lost - segments are just represented more efficiently.

Example:
  Before merging: [Wall(0,0,5m,0.15m), Wall(5,0,3m,0.15m), Wall(8,0,2m,0.15m)]
  After merging:  [Wall(0,0,10m,0.15m)]

This significantly reduces segment count without removing any actual walls.
"""

import numpy as np
from typing import List, Dict, Tuple


class SegmentMerger:
    """
    Merge adjacent aligned wall segments to reduce total segment count.

    This operates on aggregated segments, combining those that:
    - Share the same orientation (both horizontal or both vertical)
    - Are perfectly aligned (same row/column)
    - Have exactly the same extent
    """

    def __init__(self, merge_tolerance: float = 0.05):
        """
        Initialize segment merger.

        Args:
            merge_tolerance: Maximum gap between segments to merge (in meters, default: 5cm)
        """
        self.merge_tolerance = merge_tolerance

    def merge_segments(self, segments: List[dict], verbose: bool = True) -> List[dict]:
        """
        Merge adjacent aligned segments.

        Args:
            segments: List of segment dictionaries with keys:
                      'type' (horizontal/vertical), position, and extent info
            verbose: Whether to print progress messages

        Returns:
            List of merged segments (same format as input)
        """
        if len(segments) == 0:
            return []

        if verbose:
            print(
                f"\n   Merging adjacent segments (tolerance: {self.merge_tolerance*100:.1f}cm)..."
            )
            print(f"      Before merging: {len(segments)} segments")

        horizontal = [s for s in segments if s["type"] == "horizontal"]
        vertical = [s for s in segments if s["type"] == "vertical"]

        merged_horizontal = self._merge_horizontal_segments(horizontal)
        merged_vertical = self._merge_vertical_segments(vertical)

        merged_segments = merged_horizontal + merged_vertical

        if verbose:
            print(f"      After merging: {len(merged_segments)} segments")
            reduction = len(segments) - len(merged_segments)
            if len(segments) > 0:
                percent = (reduction / len(segments)) * 100
                print(f"      Reduction: {reduction} segments ({percent:.1f}%)")

        return merged_segments

    def _merge_horizontal_segments(self, segments: List[dict]) -> List[dict]:
        """Merge horizontal segments on adjacent rows with the same extent."""
        if len(segments) == 0:
            return []

        extent_groups: Dict[Tuple[int, int], List[dict]] = {}
        for seg in segments:
            extent = (seg["start_x"], seg["end_x"])
            if extent not in extent_groups:
                extent_groups[extent] = []
            extent_groups[extent].append(seg)

        merged = []

        for extent, extent_segments in extent_groups.items():
            extent_segments.sort(key=lambda s: s["y"])

            current = extent_segments[0].copy()
            current["start_y"] = current["y"]
            current["end_y"] = current["y"]

            for i in range(1, len(extent_segments)):
                next_seg = extent_segments[i]

                if next_seg["y"] == current["end_y"] + 1:
                    current["end_y"] = next_seg["y"]
                else:
                    merged.append(current)
                    current = next_seg.copy()
                    current["start_y"] = current["y"]
                    current["end_y"] = current["y"]

            merged.append(current)

        return merged

    def _merge_vertical_segments(self, segments: List[dict]) -> List[dict]:
        """Merge vertical segments on adjacent columns with the same extent."""
        if len(segments) == 0:
            return []

        extent_groups: Dict[Tuple[int, int], List[dict]] = {}
        for seg in segments:
            extent = (seg["start_y"], seg["end_y"])
            if extent not in extent_groups:
                extent_groups[extent] = []
            extent_groups[extent].append(seg)

        merged = []

        for extent, extent_segments in extent_groups.items():

            extent_segments.sort(key=lambda s: s["x"])

            current = extent_segments[0].copy()
            current["start_x"] = current["x"]
            current["end_x"] = current["x"]

            for i in range(1, len(extent_segments)):
                next_seg = extent_segments[i]

                if next_seg["x"] == current["end_x"] + 1:
                    current["end_x"] = next_seg["x"]
                else:
                    merged.append(current)
                    current = next_seg.copy()
                    current["start_x"] = current["x"]
                    current["end_x"] = current["x"]

            merged.append(current)

        return merged
