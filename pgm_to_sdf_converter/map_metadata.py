"""
Occupancy Map Metadata Parser

Parses ROS occupancy grid YAML metadata files, extracting resolution, origin,
thresholds, and image path. Includes intelligent negate flag detection to
automatically handle maps with inverted color schemes (black walls vs white walls).
"""

import yaml
import os
import numpy as np
from typing import Dict, List, Optional
from .pgm_parser import PGMParser


class MapMetadata:
    """Class to handle occupancy map metadata from YAML files."""

    def __init__(self, yaml_path: str):
        """
        Initialize MapMetadata from a YAML file.

        Args:
            yaml_path: Path to the YAML metadata file
        """
        self.yaml_path = yaml_path
        self.yaml_dir = os.path.dirname(os.path.abspath(yaml_path))

        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        # Parse standard occupancy map parameters
        self.image_path = self._resolve_image_path(data.get("image", ""))
        self.resolution = float(data.get("resolution", 0.05))  # meters per pixel
        self.origin = data.get("origin", [0.0, 0.0, 0.0])  # [x, y, theta]
        self.occupied_thresh = float(data.get("occupied_thresh", 0.65))
        self.free_thresh = float(data.get("free_thresh", 0.196))

        # We will parse the 'negate' flag, but then verify it against the actual image data.
        user_provided_negate = int(data.get("negate", 0))

        try:
            # Analyze the PGM image to determine if it has black walls or white walls.
            pgm_peek = PGMParser(self.image_path)
            image_data = pgm_peek.get_image_data()

            # Calculate the ratio of dark pixels (value < 128)
            dark_pixels = np.sum(image_data < 128)
            total_pixels = image_data.shape[0] * image_data.shape[1]
            dark_pixel_ratio = dark_pixels / total_pixels

            is_black_walls_map = dark_pixel_ratio < 0.5

            # Auto-correct the negate flag if needed.
            if user_provided_negate == 0 and is_black_walls_map:
                print(
                    "\n  [Auto-Correction] Detected a map with black walls but `negate: 0` was set."
                )
                print(
                    "  Overriding to `negate: 1` internally to ensure correct wall detection.\n"
                )
                self.negate = 1
            else:
                self.negate = user_provided_negate

        except Exception as e:
            print(f"Warning: Could not auto-detect map type due to an error: {e}")
            print("Using the `negate` value from the YAML file directly.")
            self.negate = user_provided_negate

        self._validate()

    def _resolve_image_path(self, image_path: str) -> str:
        """
        Resolve the image path relative to the YAML file location.

        Args:
            image_path: Path from YAML file

        Returns:
            Absolute path to the image file
        """
        if os.path.isabs(image_path):
            return image_path
        else:
            return os.path.join(self.yaml_dir, image_path)

    def _validate(self):
        """Validate the parsed metadata."""
        if not self.image_path:
            raise ValueError("Image path not specified in YAML file")

        if not os.path.exists(self.image_path):
            raise FileNotFoundError(f"Image file not found: {self.image_path}")

        if self.resolution <= 0:
            raise ValueError(f"Invalid resolution: {self.resolution}")

        if len(self.origin) < 2:
            raise ValueError(f"Invalid origin: {self.origin}")

    def get_origin_xy(self) -> tuple:
        """
        Get the X, Y origin coordinates.

        Returns:
            Tuple of (x, y) origin coordinates
        """
        return (self.origin[0], self.origin[1])

    def __str__(self) -> str:
        """String representation of metadata."""
        return (
            f"MapMetadata(\n"
            f"  image: {self.image_path}\n"
            f"  resolution: {self.resolution} m/pixel\n"
            f"  origin: {self.origin}\n"
            f"  negate: {self.negate} (final value)\n"
            f"  occupied_thresh: {self.occupied_thresh}\n"
            f"  free_thresh: {self.free_thresh}\n"
            f")"
        )
