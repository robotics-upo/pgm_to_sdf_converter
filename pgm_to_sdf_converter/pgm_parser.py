"""
PGM Image Parser

Parses PGM (Portable Gray Map) image files used in ROS occupancy grid maps.
Supports both P2 (ASCII) and P5 (binary) PGM formats.
"""

import numpy as np
from typing import Tuple


class PGMParser:
    """Class to parse PGM image files."""

    def __init__(self, pgm_path: str):
        """
        Initialize PGMParser and load the PGM file.

        Args:
            pgm_path: Path to the PGM file
        """
        self.pgm_path = pgm_path
        self.width = 0
        self.height = 0
        self.max_val = 0
        self.image_data = None

        self._parse()

    def _parse(self):
        """Parse the PGM file."""
        with open(self.pgm_path, "rb") as f:
            # Read magic number
            magic = f.readline().decode("ascii").strip()

            if magic not in ["P2", "P5"]:
                raise ValueError(
                    f"Unsupported PGM format: {magic}. Only P2 and P5 are supported."
                )

            # Skip comments
            line = f.readline().decode("ascii").strip()
            while line.startswith("#"):
                line = f.readline().decode("ascii").strip()

            # Read dimensions
            dimensions = line.split()
            self.width = int(dimensions[0])
            self.height = int(dimensions[1])

            # Read max value
            line = f.readline().decode("ascii").strip()
            while line.startswith("#"):
                line = f.readline().decode("ascii").strip()
            self.max_val = int(line)

            # Read image data
            if magic == "P2":
                # ASCII format
                self.image_data = self._read_ascii(f)
            elif magic == "P5":
                # Binary format
                self.image_data = self._read_binary(f)

    def _read_ascii(self, file) -> np.ndarray:
        """
        Read ASCII format PGM data.

        Args:
            file: File object positioned at image data

        Returns:
            numpy array of image data
        """
        data = []
        for line in file:
            line = line.decode("ascii").strip()
            if line and not line.startswith("#"):
                data.extend([int(x) for x in line.split()])

        return np.array(data, dtype=np.uint8).reshape((self.height, self.width))

    def _read_binary(self, file) -> np.ndarray:
        """
        Read binary format PGM data.

        Args:
            file: File object positioned at image data

        Returns:
            numpy array of image data
        """
        if self.max_val < 256:
            # Single byte per pixel
            data = np.frombuffer(file.read(), dtype=np.uint8)
        else:
            # Two bytes per pixel
            data = np.frombuffer(file.read(), dtype=np.uint16)

        return data.reshape((self.height, self.width))

    def get_image_data(self) -> np.ndarray:
        """
        Get the image data as a numpy array.

        Returns:
            numpy array of shape (height, width) with pixel values
        """
        return self.image_data

    def get_dimensions(self) -> Tuple[int, int]:
        """
        Get image dimensions.

        Returns:
            Tuple of (width, height)
        """
        return (self.width, self.height)

    def normalize_data(self) -> np.ndarray:
        """
        Get normalized image data (0.0 to 1.0).

        Returns:
            numpy array with normalized values
        """
        return self.image_data.astype(np.float32) / self.max_val

    def __str__(self) -> str:
        """String representation."""
        return (
            f"PGMParser(\n"
            f"  path: {self.pgm_path}\n"
            f"  dimensions: {self.width}x{self.height}\n"
            f"  max_value: {self.max_val}\n"
            f")"
        )
