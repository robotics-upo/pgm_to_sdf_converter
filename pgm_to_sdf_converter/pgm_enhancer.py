"""
PGM Image Enhancement Module

Enhances occupancy map images before wall extraction to reduce noise and gaps,
resulting in cleaner wall segments.

This operates directly on the PGM image file, creating a temporary enhanced version
that is then used for wall extraction. Includes adaptive parameter selection based
on map noise characteristics.
"""

import numpy as np
from scipy import ndimage
import tempfile
import os
from typing import Tuple, Dict


class PGMEnhancer:
    """
    Enhance PGM occupancy maps to reduce noise and fill gaps in walls.

    Applies morphological operations and filtering to create cleaner images
    with more continuous walls and less fragmentation.
    """

    def __init__(
        self,
        closing_size: int = 3,
        closing_iterations: int = 1,
        median_size: int = 3,
        min_component_size: int = 5,
    ):
        """
        Initialize PGM enhancer.

        Args:
            closing_size: Kernel size for morphological closing (default: 3, conservative)
            closing_iterations: Number of closing iterations (default: 1, conservative)
            median_size: Median filter kernel size (default: 3)
            min_component_size: Minimum component size to keep in pixels (default: 5, conservative)
        """
        self.closing_size = closing_size
        self.closing_iterations = closing_iterations
        self.median_size = median_size
        self.min_component_size = min_component_size

    def analyze_map_noise(self, image_data: np.ndarray) -> Dict[str, float]:
        """
        Analyze map to determine noise characteristics.

        Returns a dictionary with noise metrics:
        - small_component_ratio: Ratio of pixels in small components (< 20 pixels)
        - fragmentation_score: How fragmented the walls are
        - boundary_roughness: Average perimeter-to-area ratio (detects grainy boundaries)
        - has_grainy_boundaries: True if boundaries are rough/textured
        - noise_level: Overall noise classification (0=clean, 1=moderate, 2=heavy)
        """
        # Create binary mask: occupied pixels
        if image_data.max() > 1:
            normalized = image_data.astype(np.float32) / 255.0
        else:
            normalized = image_data.astype(np.float32)

        occupied_mask = (normalized < 0.5).astype(np.uint8)

        if np.sum(occupied_mask) == 0:
            return {
                "small_component_ratio": 0.0,
                "fragmentation_score": 0.0,
                "boundary_roughness": 0.0,
                "has_grainy_boundaries": False,
                "noise_level": 0,
            }

        # Analyze connected components
        labeled, num_components = ndimage.label(occupied_mask)

        if num_components == 0:
            return {
                "small_component_ratio": 0.0,
                "fragmentation_score": 0.0,
                "boundary_roughness": 0.0,
                "has_grainy_boundaries": False,
                "noise_level": 0,
            }

        small_component_pixels = 0
        component_sizes = []
        perimeter_to_area_ratios = []

        for label_id in range(1, num_components + 1):
            component_mask = (labeled == label_id).astype(np.uint8)
            size = np.sum(component_mask)
            component_sizes.append(size)

            if size < 20:
                small_component_pixels += size

            if size >= 20:
                eroded = ndimage.binary_erosion(component_mask)
                boundary = np.logical_xor(component_mask, eroded)
                perimeter = np.sum(boundary)

                if size > 0:
                    ratio = perimeter / np.sqrt(size)
                    perimeter_to_area_ratios.append(ratio)

        total_occupied = np.sum(occupied_mask)
        small_component_ratio = (
            small_component_pixels / total_occupied if total_occupied > 0 else 0.0
        )

        fragmentation_score = num_components / max(total_occupied / 100, 1)
        boundary_roughness = (
            np.mean(perimeter_to_area_ratios) if perimeter_to_area_ratios else 0.0
        )
        has_grainy_boundaries = boundary_roughness > 10.0

        if small_component_ratio > 0.14:
            noise_level = 2
        elif small_component_ratio > 0.10 and fragmentation_score > 5.0:
            noise_level = 1
        else:
            noise_level = 0

        return {
            "small_component_ratio": small_component_ratio,
            "fragmentation_score": fragmentation_score,
            "boundary_roughness": boundary_roughness,
            "has_grainy_boundaries": has_grainy_boundaries,
            "noise_level": noise_level,
            "num_components": num_components,
            "total_occupied": total_occupied,
        }

    def get_adaptive_parameters(
        self, noise_metrics: Dict[str, float]
    ) -> Dict[str, int]:
        """
        Select enhancement parameters based on noise analysis.

        Args:
            noise_metrics: Output from analyze_map_noise()

        Returns:
            Dictionary with recommended parameters
        """
        noise_level = noise_metrics["noise_level"]
        has_grainy_boundaries = noise_metrics.get("has_grainy_boundaries", False)

        if noise_level == 0:
            if has_grainy_boundaries:
                return {
                    "closing_size": 4,
                    "closing_iterations": 1,
                    "opening_size": 0,
                    "opening_iterations": 0,
                    "min_component_size": 0,
                    "apply_enhancement": True,
                    "enhancement_type": "boundary_smoothing",
                }
            else:
                return {
                    "closing_size": 0,
                    "closing_iterations": 0,
                    "opening_size": 0,
                    "opening_iterations": 0,
                    "min_component_size": 3,
                    "apply_enhancement": False,
                    "enhancement_type": "none",
                }
        elif noise_level == 1:
            return {
                "closing_size": 3,
                "closing_iterations": 1,
                "opening_size": 0,
                "opening_iterations": 0,
                "min_component_size": 5,
                "apply_enhancement": True,
                "enhancement_type": "moderate",
            }
        else:
            return {
                "closing_size": 7,
                "closing_iterations": 3,
                "opening_size": 0,
                "opening_iterations": 0,
                "min_component_size": 15,
                "apply_enhancement": True,
                "enhancement_type": "aggressive",
            }

    def enhance_pgm(
        self, image_data: np.ndarray, verbose: bool = True, adaptive: bool = False
    ) -> Tuple[np.ndarray, bool]:
        """
        Enhance PGM image to reduce noise and fill gaps.

        Args:
            image_data: Original image data (0-255 grayscale)
            verbose: Print progress information
            adaptive: Use adaptive parameter selection based on map noise analysis

        Returns:
            Tuple of (enhanced_image_data, was_enhanced)
            - enhanced_image_data: Enhanced image data (0-255 grayscale)
            - was_enhanced: True if enhancement was applied, False if skipped
        """
        if verbose:
            print("\n" + "─" * 70)
            print("  IMAGE ENHANCEMENT")
            print("─" * 70)

        # Adaptive parameter selection
        enhancement_type = "standard"

        if adaptive:
            noise_metrics = self.analyze_map_noise(image_data)
            adaptive_params = self.get_adaptive_parameters(noise_metrics)

            if verbose:
                noise_level_names = ["CLEAN", "MODERATE", "HEAVY"]
                noise_name = noise_level_names[noise_metrics["noise_level"]]

                if noise_metrics["has_grainy_boundaries"]:
                    print(f"\n  Map Type:      {noise_name} (grainy boundaries)")
                else:
                    print(f"\n  Map Type:      {noise_name}")

            if not adaptive_params["apply_enhancement"]:
                if verbose:
                    print(f"  Action:        No enhancement needed\n")
                return image_data.copy(), False

            self.closing_size = adaptive_params["closing_size"]
            self.closing_iterations = adaptive_params["closing_iterations"]
            self.min_component_size = adaptive_params["min_component_size"]
            enhancement_type = adaptive_params.get("enhancement_type", "standard")

            if verbose:
                if enhancement_type == "boundary_smoothing":
                    print(f"  Strategy:      Boundary smoothing + gap filling")
                else:
                    print(f"  Strategy:      Noise reduction + gap filling")

        enhanced = image_data.copy()
        occupied_mask = (enhanced < 128).astype(np.uint8)

        if enhancement_type == "boundary_smoothing":
            pad_size = max(self.closing_size, 3)
            padded_mask = np.pad(
                occupied_mask, pad_size, mode="constant", constant_values=0
            )

            kernel = np.ones((self.closing_size, self.closing_size), dtype=np.uint8)
            smoothed_padded = ndimage.binary_closing(
                padded_mask, structure=kernel, iterations=self.closing_iterations
            ).astype(np.uint8)

            smoothed = smoothed_padded[pad_size:-pad_size, pad_size:-pad_size]
            pixels_smoothed = int(np.sum(smoothed)) - int(np.sum(occupied_mask))

            if self.min_component_size > 0:
                cleaned = self._remove_small_components(
                    smoothed, self.min_component_size
                )
                noise_removed = int(np.sum(smoothed)) - int(np.sum(cleaned))
            else:
                cleaned = smoothed
                noise_removed = 0

            enhanced = np.where(cleaned, 0, 254).astype(np.uint8)

            if verbose:
                print(f"  Gaps filled:   {pixels_smoothed} pixels")
                if noise_removed > 0:
                    print(f"  Noise removed: {noise_removed} pixels")
                else:
                    print(f"  Features:      All preserved")
                print()

        else:
            enhanced = ndimage.median_filter(enhanced, size=self.median_size)
            occupied_mask = (enhanced < 128).astype(np.uint8)

            pad_size = max(self.closing_size, 3)
            padded_mask = np.pad(
                occupied_mask, pad_size, mode="constant", constant_values=0
            )

            kernel = np.ones((self.closing_size, self.closing_size), dtype=np.uint8)
            closed_padded = ndimage.binary_closing(
                padded_mask, structure=kernel, iterations=self.closing_iterations
            ).astype(np.uint8)

            closed = closed_padded[pad_size:-pad_size, pad_size:-pad_size]
            gaps_filled = int(np.sum(closed)) - int(np.sum(occupied_mask))

            cleaned = self._remove_small_components(closed, self.min_component_size)
            noise_removed = int(np.sum(closed)) - int(np.sum(cleaned))

            enhanced = np.where(cleaned, 0, 254).astype(np.uint8)

            if verbose:
                print(f"  Gaps filled:   {gaps_filled} pixels")
                print(f"  Noise removed: {noise_removed} pixels")
                print()

        return enhanced, True

    def _remove_small_components(
        self, binary_map: np.ndarray, min_size: int
    ) -> np.ndarray:
        """
        Remove small isolated components from binary map.

        Args:
            binary_map: Binary image (0 or 1)
            min_size: Minimum component size to keep

        Returns:
            Cleaned binary image
        """
        labeled, num_components = ndimage.label(binary_map)

        if num_components == 0:
            return binary_map

        clean_map = np.zeros_like(binary_map)

        for label_id in range(1, num_components + 1):
            component_size = np.sum(labeled == label_id)
            if component_size >= min_size:
                clean_map[labeled == label_id] = 1

        return clean_map.astype(np.uint8)

    def save_enhanced_pgm(
        self, original_path: str, enhanced_data: np.ndarray, temp_dir: str = None
    ) -> str:
        """
        Save enhanced image as temporary PGM file.

        Args:
            original_path: Path to original PGM file
            enhanced_data: Enhanced image data
            temp_dir: Directory for temporary file (default: system temp)

        Returns:
            Path to temporary enhanced PGM file
        """
        # Create temporary file with similar name
        original_name = os.path.basename(original_path)
        name_without_ext = os.path.splitext(original_name)[0]

        if temp_dir is None:
            temp_dir = tempfile.gettempdir()

        temp_path = os.path.join(temp_dir, f"{name_without_ext}_enhanced.pgm")

        # Write PGM file (P5 format - binary grayscale)
        height, width = enhanced_data.shape

        with open(temp_path, "wb") as f:
            # PGM header
            f.write(b"P5\n")
            f.write(f"{width} {height}\n".encode())
            f.write(b"255\n")
            # Image data
            f.write(enhanced_data.tobytes())

        return temp_path
