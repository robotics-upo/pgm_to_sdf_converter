#!/usr/bin/env python3
"""
PGM to SDF Converter - Main Entry Point

Converts ROS occupancy grid maps (PGM format) to Gazebo SDF world files with
rectangular wall segments. Features adaptive image enhancement, intelligent wall
extraction, and segment merging for optimal representation.

Usage:
    ros2 run pgm_to_sdf_converter pgm_to_sdf <map.yaml> [options]

Example:
    ros2 run pgm_to_sdf_converter pgm_to_sdf bookstore.yaml -o bookstore.sdf

The converter automatically:
- Analyzes map quality and applies adaptive enhancement if needed
- Extracts walls using two-pass horizontal/vertical aggregation
- Merges adjacent segments to reduce Gazebo object count
- Preserves all map features while optimizing representation
"""

import sys
import os
import argparse
from .map_metadata import MapMetadata
from .pgm_parser import PGMParser
from .wall_aggregation_extractor import WallAggregationExtractor
from .sdf_generator import SDFGenerator
from .pgm_enhancer import PGMEnhancer


class PGMToSDFConverter:
    """
    Main converter class for PGM to SDF conversion.

    Orchestrates the complete conversion pipeline:
    1. Parse YAML metadata and PGM image
    2. Apply adaptive image enhancement (noise reduction, gap filling)
    3. Extract wall segments using two-pass aggregation
    4. Merge adjacent segments for optimization
    5. Generate Gazebo SDF world file
    """

    def __init__(self):
        """Initialize converter."""
        self.metadata = None
        self.pgm_parser = None
        self.wall_extractor = None
        self.sdf_generator = None

    def convert(
        self,
        yaml_path: str,
        output_path: str = None,
        wall_height: float = 2.5,
        min_wall_size: int = 1,
        min_wall_length: float = 0.5,
        world_name: str = "occupancy_map_world",
        enable_merging: bool = True,
        merge_tolerance: float = 0.05,
        keep_cleaned_pgm: bool = False,
    ) -> str:
        """
        Convert occupancy map to SDF world file.

        Args:
            yaml_path: Path to YAML metadata file
            output_path: Output SDF file path (optional)
            wall_height: Height of walls in meters
            min_wall_size: Minimum wall size in pixels
            min_wall_length: Minimum wall length in meters (default: 0.5)
            world_name: Name of the world
            enable_merging: Enable segment merging to combine adjacent segments
            merge_tolerance: Maximum gap between segments to merge (meters)
            keep_cleaned_pgm: Keep the cleaned PGM file when enhancement is applied (default: False)

        Returns:
            SDF content as string
        """
        enhanced_pgm_path = None
        self.keep_cleaned_pgm = keep_cleaned_pgm
        print("\n" + "=" * 70)
        print("  PGM TO SDF CONVERTER")
        print("=" * 70)
        print(f"\n📂 Input: {yaml_path}")

        self.metadata = MapMetadata(yaml_path)

        pgm_path_to_use = self.metadata.image_path
        try:
            temp_parser = PGMParser(self.metadata.image_path)
            original_data = temp_parser.get_image_data()

            enhancer = PGMEnhancer()
            enhanced_data, was_enhanced = enhancer.enhance_pgm(
                original_data, verbose=True, adaptive=True
            )

            if was_enhanced:
                if keep_cleaned_pgm:
                    original_dir = os.path.dirname(self.metadata.image_path)
                    enhanced_pgm_path = enhancer.save_enhanced_pgm(
                        self.metadata.image_path, enhanced_data, temp_dir=original_dir
                    )
                else:
                    enhanced_pgm_path = enhancer.save_enhanced_pgm(
                        self.metadata.image_path, enhanced_data
                    )
                pgm_path_to_use = enhanced_pgm_path

        except Exception as e:
            print(f"\n⚠️  Enhancement failed: {e}")
            print(f"   → Using original image\n")
            pgm_path_to_use = self.metadata.image_path

        self.pgm_parser = PGMParser(pgm_path_to_use)

        print(f"\n📊 Map Properties:")
        print(f"   Resolution:  {self.metadata.resolution} m/pixel")
        print(
            f"   Size:        {self.pgm_parser.width} × {self.pgm_parser.height} pixels"
        )
        print(
            f"   Origin:      ({self.metadata.get_origin_xy()[0]:.2f}, {self.metadata.get_origin_xy()[1]:.2f}) m"
        )

        self.wall_extractor = WallAggregationExtractor(
            image_data=self.pgm_parser.get_image_data(),
            resolution=self.metadata.resolution,
            origin=self.metadata.get_origin_xy(),
            occupied_thresh=self.metadata.occupied_thresh,
            free_thresh=self.metadata.free_thresh,
            negate=self.metadata.negate,
            wall_height=wall_height,
            enable_merging=enable_merging,
            merge_tolerance=merge_tolerance,
        )

        walls = self.wall_extractor.extract_walls()

        print("\n🔧 Generating SDF world file...")
        self.sdf_generator = SDFGenerator(world_name=world_name)
        sdf_content = self.sdf_generator.generate(
            walls=walls, ground_size=None, wall_height=wall_height
        )

        if output_path is None:
            yaml_base = os.path.splitext(yaml_path)[0]
            output_path = yaml_base + ".sdf"

        self.sdf_generator.save_to_file(sdf_content, output_path)

        print("\n" + "=" * 70)
        print("  ✓ CONVERSION COMPLETE")
        print("=" * 70)
        print(f"\n📁 Output:  {output_path}")

        if enhanced_pgm_path is not None:
            if keep_cleaned_pgm:
                print(f"💾 Cleaned: {enhanced_pgm_path}")
            else:
                try:
                    os.remove(enhanced_pgm_path)
                except Exception:
                    pass

        print()

        return sdf_content


def main(args=None):
    """
    Main entry point for the converter.

    Args:
        args: Command line arguments (optional, for testing)
    """
    parser = argparse.ArgumentParser(
        description="Convert occupancy map (PGM + YAML) to Gazebo SDF world file",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert map (adaptive enhancement always enabled)
  ros2 run pgm_to_sdf_converter pgm_to_sdf map.yaml
  ros2 run pgm_to_sdf_converter pgm_to_sdf map

  # Convert with custom output filename
  ros2 run pgm_to_sdf_converter pgm_to_sdf map -o output.sdf

  # Convert with custom wall height
  ros2 run pgm_to_sdf_converter pgm_to_sdf map --wall-height 3.0

  # Keep the cleaned PGM file for inspection (only saved if enhancement is applied)
  ros2 run pgm_to_sdf_converter pgm_to_sdf map.yaml --keep-cleaned-pgm

Note:
  ADAPTIVE ENHANCEMENT is ALWAYS ENABLED - the system automatically analyzes each map
  and applies appropriate enhancement based on noise characteristics:
  - Clean maps: No enhancement (preserves all details)
  - Moderate noise: Conservative enhancement
  - Heavy noise (SLAM): Aggressive enhancement

  Segment merging is also ENABLED BY DEFAULT - adjacent aligned segments are automatically
  combined to reduce segment count without losing wall data.
        """,
    )

    parser.add_argument(
        "yaml_file",
        type=str,
        help="Path to occupancy map YAML metadata file (with or without .yaml extension)",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Output SDF file path (default: same as input YAML with .sdf extension)",
    )

    parser.add_argument(
        "--wall-height",
        type=float,
        default=2.5,
        help="Height of walls in meters (default: 2.5)",
    )

    parser.add_argument(
        "--min-wall-size",
        type=int,
        default=1,
        help="Minimum wall size in pixels (default: 1)",
    )

    parser.add_argument(
        "--min-wall-length",
        type=float,
        default=0.5,
        help="Minimum wall length in meters (default: 0.5, use 1.0 for corridor maps)",
    )

    parser.add_argument(
        "--world-name",
        type=str,
        default="occupancy_map_world",
        help="Name of the generated world (default: occupancy_map_world)",
    )

    parser.add_argument(
        "--no-merge",
        action="store_true",
        help="Disable segment merging (merging is enabled by default to reduce segment count)",
    )

    parser.add_argument(
        "--merge-tolerance",
        type=float,
        default=0.05,
        metavar="METERS",
        help="Maximum gap between segments to merge in meters (default: 0.05m = 5cm)",
    )

    parser.add_argument(
        "--keep-cleaned-pgm",
        action="store_true",
        help="Keep the cleaned PGM file (saves next to original instead of deleting)",
    )

    # Parse arguments
    if args is None:
        args = parser.parse_args()
    else:
        args = parser.parse_args(args)

    yaml_file = args.yaml_file
    if not yaml_file.endswith(".yaml") and not yaml_file.endswith(".yml"):
        yaml_file_with_ext = yaml_file + ".yaml"
        if os.path.exists(yaml_file_with_ext):
            yaml_file = yaml_file_with_ext
        else:
            yaml_file_with_yml = yaml_file + ".yml"
            if os.path.exists(yaml_file_with_yml):
                yaml_file = yaml_file_with_yml

    if not os.path.exists(yaml_file):
        print(f"Error: YAML file not found: {yaml_file}", file=sys.stderr)
        if yaml_file != args.yaml_file:
            print(f"       Also tried: {args.yaml_file}", file=sys.stderr)
        return 1

    try:
        converter = PGMToSDFConverter()
        converter.convert(
            yaml_path=yaml_file,
            output_path=args.output,
            wall_height=args.wall_height,
            min_wall_size=args.min_wall_size,
            min_wall_length=args.min_wall_length,
            world_name=args.world_name,
            enable_merging=not args.no_merge,
            merge_tolerance=args.merge_tolerance,
            keep_cleaned_pgm=args.keep_cleaned_pgm,
        )
        return 0

    except Exception as e:
        print(f"Error during conversion: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
