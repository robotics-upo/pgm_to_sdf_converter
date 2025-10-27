"""
Gazebo SDF World File Generator

Generates Gazebo SDF (Simulation Description Format) world files from wall segments.
Creates properly formatted XML with physics, lighting, and collision properties
for simulation in Gazebo.
"""

from typing import List, Tuple
from xml.etree import ElementTree as ET
from xml.dom import minidom
from .wall_aggregation_extractor import Wall


class SDFGenerator:
    """Class to generate Gazebo SDF world files."""

    def __init__(self, world_name: str = "occupancy_map_world"):
        """
        Initialize SDFGenerator.

        Args:
            world_name: Name of the world
        """
        self.world_name = world_name
        self.sdf_version = "1.7"

    def generate(
        self,
        walls: List[Wall],
        ground_size: Tuple[float, float] = None,
        wall_height: float = 2.5,
        wall_thickness: float = 0.15,
    ) -> str:
        """
        Generate SDF world file content.

        Args:
            walls: List of Wall objects
            ground_size: Size of ground plane (width, length). If None, calculated from walls
            wall_height: Height of walls in meters
            wall_thickness: Thickness of walls in meters

        Returns:
            SDF XML content as string
        """
        # Create root SDF element
        sdf = ET.Element("sdf", version=self.sdf_version)

        # Create world element
        world = ET.SubElement(sdf, "world", name=self.world_name)

        # Add physics
        self._add_physics(world)

        # Add lighting
        self._add_sun(world)

        # Add ground plane
        self._add_ground_plane(world, ground_size, walls)

        # Add all walls
        for i, wall in enumerate(walls):
            self._add_wall(world, wall, i, wall_height, wall_thickness)

        # Convert to pretty-printed string
        return self._prettify(sdf)

    def _add_physics(self, world: ET.Element):
        """Add physics configuration to world."""
        physics = ET.SubElement(
            world, "physics", name="default_physics", default="0", type="ode"
        )

        # Max step size
        max_step_size = ET.SubElement(physics, "max_step_size")
        max_step_size.text = "0.001"

        # Real time factor
        real_time_factor = ET.SubElement(physics, "real_time_factor")
        real_time_factor.text = "1"

        # Real time update rate
        real_time_update_rate = ET.SubElement(physics, "real_time_update_rate")
        real_time_update_rate.text = "1000"

    def _add_sun(self, world: ET.Element):
        """Add sun light source to world."""
        light = ET.SubElement(world, "light", name="sun", type="directional")

        # Cast shadows
        cast_shadows = ET.SubElement(light, "cast_shadows")
        cast_shadows.text = "1"

        # Pose
        pose = ET.SubElement(light, "pose")
        pose.text = "0 0 10 0 0 0"

        # Diffuse color (white)
        diffuse = ET.SubElement(light, "diffuse")
        diffuse.text = "0.8 0.8 0.8 1"

        # Specular color
        specular = ET.SubElement(light, "specular")
        specular.text = "0.2 0.2 0.2 1"

        # Attenuation
        attenuation = ET.SubElement(light, "attenuation")
        range_elem = ET.SubElement(attenuation, "range")
        range_elem.text = "1000"
        constant = ET.SubElement(attenuation, "constant")
        constant.text = "0.9"
        linear = ET.SubElement(attenuation, "linear")
        linear.text = "0.01"
        quadratic = ET.SubElement(attenuation, "quadratic")
        quadratic.text = "0.001"

        # Direction
        direction = ET.SubElement(light, "direction")
        direction.text = "-0.5 0.1 -0.9"

    def _add_ground_plane(
        self, world: ET.Element, ground_size: Tuple[float, float], walls: List[Wall]
    ):
        """Add ground plane to world."""
        # Calculate ground size if not provided
        if ground_size is None:
            if walls:
                # Calculate bounding box from walls
                min_x = min(w.x - w.width / 2 for w in walls)
                max_x = max(w.x + w.width / 2 for w in walls)
                min_y = min(w.y - w.length / 2 for w in walls)
                max_y = max(w.y + w.length / 2 for w in walls)

                # Add 20% margin
                margin = 0.2
                width = (max_x - min_x) * (1 + margin)
                length = (max_y - min_y) * (1 + margin)

                # Center position
                center_x = (min_x + max_x) / 2
                center_y = (min_y + max_y) / 2
            else:
                width, length = 20.0, 20.0
                center_x, center_y = 0.0, 0.0
        else:
            width, length = ground_size
            center_x, center_y = 0.0, 0.0

        # Create ground plane model
        model = ET.SubElement(world, "model", name="ground_plane")

        # Static
        static = ET.SubElement(model, "static")
        static.text = "true"

        # Link
        link = ET.SubElement(model, "link", name="link")

        # Collision
        collision = ET.SubElement(link, "collision", name="collision")
        coll_geometry = ET.SubElement(collision, "geometry")
        plane = ET.SubElement(coll_geometry, "plane")
        normal = ET.SubElement(plane, "normal")
        normal.text = "0 0 1"
        size = ET.SubElement(plane, "size")
        size.text = f"{width} {length}"

        # Visual
        visual = ET.SubElement(link, "visual", name="visual")
        vis_geometry = ET.SubElement(visual, "geometry")
        vis_plane = ET.SubElement(vis_geometry, "plane")
        vis_normal = ET.SubElement(vis_plane, "normal")
        vis_normal.text = "0 0 1"
        vis_size = ET.SubElement(vis_plane, "size")
        vis_size.text = f"{width} {length}"

        # Material
        material = ET.SubElement(visual, "material")
        script = ET.SubElement(material, "script")
        script_uri = ET.SubElement(script, "uri")
        script_uri.text = "file://media/materials/scripts/gazebo.material"
        script_name = ET.SubElement(script, "name")
        script_name.text = "Gazebo/Grey"

    def _add_wall(
        self, world: ET.Element, wall: Wall, index: int, height: float, thickness: float
    ):
        """
        Add a wall model to the world.

        Args:
            world: World element
            wall: Wall object
            index: Wall index for unique naming
            height: Wall height in meters
            thickness: Wall thickness (added to thinner dimension)
        """
        model = ET.SubElement(world, "model", name=f"wall_{index}")

        # Static
        static = ET.SubElement(model, "static")
        static.text = "true"

        # Pose
        pose = ET.SubElement(model, "pose")
        pose.text = f"{wall.x} {wall.y} {height/2} 0 0 {wall.angle}"

        # Link
        link = ET.SubElement(model, "link", name="link")

        # Determine box dimensions
        # The wall already has width and length from extraction
        # We add thickness to make proper 3D boxes
        box_x = wall.width
        box_y = wall.length
        box_z = height

        # Collision
        collision = ET.SubElement(link, "collision", name="collision")
        coll_geometry = ET.SubElement(collision, "geometry")
        box = ET.SubElement(coll_geometry, "box")
        size = ET.SubElement(box, "size")
        size.text = f"{box_x} {box_y} {box_z}"

        # Visual
        visual = ET.SubElement(link, "visual", name="visual")
        vis_geometry = ET.SubElement(visual, "geometry")
        vis_box = ET.SubElement(vis_geometry, "box")
        vis_size = ET.SubElement(vis_box, "size")
        vis_size.text = f"{box_x} {box_y} {box_z}"

        # Material
        material = ET.SubElement(visual, "material")
        script = ET.SubElement(material, "script")
        script_uri = ET.SubElement(script, "uri")
        script_uri.text = "file://media/materials/scripts/gazebo.material"
        script_name = ET.SubElement(script, "name")
        script_name.text = "Gazebo/White"

    def _prettify(self, elem: ET.Element) -> str:
        """
        Return a pretty-printed XML string.

        Args:
            elem: XML element

        Returns:
            Formatted XML string
        """
        rough_string = ET.tostring(elem, encoding="utf-8")
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ", encoding="utf-8").decode("utf-8")

    def save_to_file(self, content: str, filepath: str):
        """
        Save SDF content to file.

        Args:
            content: SDF XML content
            filepath: Output file path
        """
        with open(filepath, "w") as f:
            f.write(content)
