# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from __future__ import annotations

import itertools
import logging
import re
from collections.abc import Sequence
from typing import TYPE_CHECKING, Any

import numpy as np
import torch

import isaacsim.core.utils.torch as torch_utils
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Usd, UsdGeom, UsdPhysics

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.markers import VisualizationMarkers
from isaaclab.sensors.sensor_base import SensorBase

if TYPE_CHECKING:
    from .tactile_array_sensor_cfg import TactileArraySensorCfg

import trimesh

from .tactile_array_sensor_data import TactileArraySensorData

logger = logging.getLogger(__name__)


class TactileArraySensor(SensorBase):
    r"""A tactile sensor for tactile array finger.

    **Example Usage:**
        For a complete working example, see: ``examples/tactile_array_sensor.py``

    **Current Limitations:**
        - SDF collision meshes must be pre-computed and objects specified before simulation starts
        - Force field computation requires specific rigid body and mesh configurations
        - No support for dynamic addition/removal of interacting objects during runtime

    Configuration Requirements:
        The following requirements must be satisfied for proper sensor operation:

        **Force Field Computation**
            If ``enable_force_field=True``, the following parameters are required:

            * ``contact_object_prim_path_expr`` - Prim path expression to find the contact object prim

        **SDF Computation**
            When force field computation is enabled, penalty-based normal and shear forces are
            computed using Signed Distance Field (SDF) queries. To achieve GPU acceleration:

            * Interacting objects should have pre-computed SDF collision meshes
            * An SDFView must be defined during initialization, therefore interacting objects
              should be specified before simulation.

    """

    cfg: TactileArraySensorCfg
    """The configuration parameters."""

    def __init__(self, cfg: TactileArraySensorCfg):
        """Initializes the tactile sensor object.

        Args:
            cfg: The configuration parameters.
        """

        # Create empty variables for storing output data
        self._data = TactileArraySensorData()

        # Force field tactile sensing
        self._tactile_pos_local: torch.Tensor | None = None
        self._tactile_quat_local: torch.Tensor | None = None
        self._contact_object_prim_path_expr: str | None = None

        # COMs for velocity correction
        self._elastomer_com_b: torch.Tensor | None = None
        self._contact_object_com_b: torch.Tensor | None = None

        # Physics views
        self._physics_sim_view = None
        self._elastomer_body_view = None
        self._contact_object_body_view = None
        self._contact_object_scale = None

        # Visualization
        self._tactile_visualizer: VisualizationMarkers | None = None

        # Tactile points count
        self.num_tactile_points: int = 0
        self.expected_num_tactile_points: int = 0
        self.num_valid_tactile_points: int = 0
        self._valid_tactile_mask_local: torch.Tensor | None = None

        # Now call parent class constructor
        super().__init__(cfg)

    def __str__(self) -> str:
        """Returns: A string containing information about the instance."""
        return (
            f"Tactile sensor @ '{self.cfg.prim_path}': \n"
            f"\tupdate period (s) : {self.cfg.update_period}\n"
            f"\tforce field enabled: {self.cfg.enable_force_field}\n"
            f"\tnum instances     : {self.num_instances}\n"
        )

    """
    Properties
    """

    @property
    def num_instances(self) -> int:
        return self._num_envs

    @property
    def data(self) -> TactileArraySensorData:
        # Update sensors if needed
        self._update_outdated_buffers()
        # Return the data
        return self._data

    """
    Implementation
    """

    def _initialize_impl(self):
        """Initializes the sensor-related handles and internal buffers."""
        super()._initialize_impl()

        # InteractiveScene only expands placeholders for sensor types that ship in Isaac Lab.
        # Resolve this extension's contact path here so the official 2.3.2 core stays untouched.
        self._contact_object_prim_path_expr = self._resolve_contact_object_prim_path_expr()

        # Obtain global simulation view
        self._physics_sim_view = SimulationManager.get_physics_sim_view()

        # Initialize force field tactile sensing
        if self.cfg.enable_force_field:
            self._initialize_force_field()

        # Initialize visualization
        if self.cfg.debug_vis:
            self._initialize_visualization()

    def _resolve_contact_object_prim_path_expr(self) -> str | None:
        """Resolve ``{ENV_REGEX_NS}`` without patching :class:`InteractiveScene`.

        ``InteractiveScene`` has already expanded :attr:`cfg.prim_path` before this sensor is
        constructed.  The resolved sensor path therefore contains the scene's environment
        namespace (normally ``/World/envs/env_.*``), which is reused for the contact object.
        A concrete ``env_0`` parent path is used as a fallback when the sensor is initialized
        outside the normal scene construction path.
        """
        expression = self.cfg.contact_object_prim_path_expr
        if expression is None or "{ENV_REGEX_NS}" not in expression:
            return expression

        namespace_match = re.match(r"^(?P<namespace>.*?/env_[^/]+)(?:/|$)", self.cfg.prim_path)
        if namespace_match is not None:
            env_regex_ns = namespace_match.group("namespace")
            # A directly constructed sensor may carry a concrete env_0 path.  Promote it
            # to the scene-wide expression so all matched sensor instances share one view.
            env_regex_ns = re.sub(r"(?<=/env_)\d+$", ".*", env_regex_ns)
        else:
            concrete_parent_path = self._parent_prims[0].GetPath().pathString if len(self._parent_prims) > 0 else ""
            concrete_match = re.match(r"^(?P<prefix>.*?/env_)\d+(?:/|$)", concrete_parent_path)
            if concrete_match is None:
                raise RuntimeError(
                    "Unable to resolve {ENV_REGEX_NS} in contact_object_prim_path_expr from "
                    f"sensor path {self.cfg.prim_path!r}. Use an explicit contact-object path "
                    "when the sensor is outside /World/envs/env_<index>."
                )
            env_regex_ns = concrete_match.group("prefix") + ".*"

        return expression.replace("{ENV_REGEX_NS}", env_regex_ns)

    def _initialize_force_field(self):
        """Initialize force field tactile sensing components.

        This method sets up all components required for force field based tactile sensing:

        1. Creates PhysX views for elastomer and contact object rigid bodies
        2. Generates tactile sensing points on the elastomer surface using mesh geometry
        3. Initializes SDF (Signed Distance Field) for collision detection
        4. Creates data buffers for storing force field measurements

        The tactile points are generated by ray-casting onto the elastomer mesh surface
        to create a grid of sensing points that will be used for force computation.

        """

        # Generate tactile points on elastomer surface
        self._generate_tactile_points(
            num_divs=list(self.cfg.tactile_array_size),
            tactile_points_distance=getattr(self.cfg, "tactile_points_distance", 0.001),
            visualize=self.cfg.trimesh_vis_tactile_points,
        )

        self._create_physx_views()

        # Initialize force field data buffers
        self._initialize_force_field_buffers()
        logger.info("Force field tactile sensing initialized.")

    def _create_physx_views(self) -> None:
        """Create PhysX views for contact object and elastomer bodies.

        This method sets up the necessary PhysX views for force field computation:
        1. Creates rigid body view for elastomer
        2. If contact object prim path expression is not None, then:
            a. Finds and validates the object prim and its collision mesh
            b. Creates SDF view for collision detection
            c. Creates rigid body view for object

        """
        # PhysX tensor views use glob-style expressions (``env_*``), not USD regex.
        elastomer_pattern = self._parent_prims[0].GetPath().pathString.replace("env_0", "env_*")
        self._elastomer_body_view = self._physics_sim_view.create_rigid_body_view([elastomer_pattern])
        # Get elastomer COM for velocity correction
        self._elastomer_com_b = self._elastomer_body_view.get_coms().to(self._device).split([3, 4], dim=-1)[0]

        if self._contact_object_prim_path_expr is None:
            return

        contact_object_mesh, contact_object_rigid_body = self._find_contact_object_components()
        # Create SDF view for collision detection
        num_query_points = self.cfg.tactile_array_size[0] * self.cfg.tactile_array_size[1]
        mesh_path_pattern = contact_object_mesh.GetPath().pathString.replace("env_0", "env_*")
        self._contact_object_sdf_view = self._physics_sim_view.create_sdf_shape_view(
            mesh_path_pattern, num_query_points
        )

        # Create rigid body views for contact object and elastomer
        body_path_pattern = contact_object_rigid_body.GetPath().pathString.replace("env_0", "env_*")
        self._contact_object_body_view = self._physics_sim_view.create_rigid_body_view([body_path_pattern])
        # Isaac Lab's stage query helper uses regex-style expressions.
        mesh_prims = sim_utils.find_matching_prims(mesh_path_pattern.replace("env_*", "env_.*"))
        self._contact_object_scale = torch.tensor(
            [sim_utils.resolve_prim_scale(prim) for prim in mesh_prims],
            dtype=torch.float32,
            device=self._device,
        )
        # Get contact object COM for velocity correction
        self._contact_object_com_b = self._contact_object_body_view.get_coms().to(self._device).split([3, 4], dim=-1)[0]

    def _find_contact_object_components(self) -> tuple[Any, Any]:
        """Find and validate contact object SDF mesh and its parent rigid body.

        This method searches for the contact object prim using the configured filter pattern,
        then locates the first SDF collision mesh within that prim hierarchy and
        identifies its parent rigid body for physics simulation.

        Returns:
            Tuple of (contact_object_mesh, contact_object_rigid_body)
            Returns None if contact object components are not found.

        Note:
            Only SDF meshes are supported for optimal force field computation performance.
            If no SDF mesh is found, the method will log a warning and return None.
        """
        # Find the contact object prim using the configured pattern
        contact_object_prim = sim_utils.find_first_matching_prim(self._contact_object_prim_path_expr)
        if contact_object_prim is None:
            raise RuntimeError(f"No contact object prim found matching pattern: {self._contact_object_prim_path_expr}")

        def is_sdf_mesh(prim: Usd.Prim) -> bool:
            """Check if a mesh prim is configured for SDF approximation."""
            return (
                prim.HasAPI(UsdPhysics.MeshCollisionAPI)
                and UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == "sdf"
            )

        # Find the SDF mesh within the contact object
        contact_object_mesh = sim_utils.get_first_matching_child_prim(
            contact_object_prim.GetPath(), predicate=is_sdf_mesh
        )
        if contact_object_mesh is None:
            raise RuntimeError(
                f"No SDF mesh found under contact object at path: {contact_object_prim.GetPath().pathString}"
            )

        def find_parent_rigid_body(prim: Usd.Prim) -> Usd.Prim | None:
            """Find the first parent prim with RigidBodyAPI."""
            current_prim = prim
            while current_prim and current_prim.IsValid():
                if current_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    return current_prim
                current_prim = current_prim.GetParent()
                if current_prim.GetPath() == "/":
                    break
            return None

        # Find the rigid body parent of the SDF mesh
        contact_object_rigid_body = find_parent_rigid_body(contact_object_mesh)
        if contact_object_rigid_body is None:
            raise RuntimeError(
                f"No contact object rigid body found for mesh at path: {contact_object_mesh.GetPath().pathString}"
            )

        return contact_object_mesh, contact_object_rigid_body

    def _get_contact_object_scales(self) -> torch.Tensor:
        """Return cached world scales of the SDF mesh prims."""
        if self._contact_object_scale is None:
            return torch.ones((self._num_envs, 3), dtype=torch.float32, device=self._device)
        return self._contact_object_scale

    def _generate_tactile_points(self, num_divs: list, tactile_points_distance: float, visualize: bool):
        """Generate tactile sensing points from elastomer mesh geometry.

        This method creates a grid of tactile sensing points on the elastomer surface
        by ray-casting onto the mesh geometry. Visual meshes are used for smoother point sampling.

        Args:
            num_divs: Number of divisions [rows, cols] for the tactile grid.
            tactile_points_distance: Distance between adjacent tactile points in meters.
            visualize: Whether to show the generated points in trimesh visualization.

        """

        # Get the elastomer prim path
        elastomer_prim_path = self._parent_prims[0].GetPath().pathString

        def is_visual_mesh(prim) -> bool:
            """Check if a mesh prim has visual properties (visual mesh, not collision mesh)."""
            return prim.IsA(UsdGeom.Mesh) and not prim.HasAPI(UsdPhysics.CollisionAPI)

        elastomer_mesh_prim = sim_utils.get_first_matching_child_prim(elastomer_prim_path, predicate=is_visual_mesh)
        if elastomer_mesh_prim is None:
            raise RuntimeError(f"No visual mesh found under elastomer at path: {elastomer_prim_path}")

        logger.info(f"Generating tactile points from USD mesh: {elastomer_mesh_prim.GetPath().pathString}")

        # Extract mesh data
        usd_mesh = UsdGeom.Mesh(elastomer_mesh_prim)
        points = np.asarray(usd_mesh.GetPointsAttr().Get())
        face_indices = np.asarray(usd_mesh.GetFaceVertexIndicesAttr().Get())

        # Simple triangulation
        faces = face_indices.reshape(-1, 3)

        # Create bounds
        mesh_bounds = np.array([points.min(axis=0), points.max(axis=0)])

        # Create trimesh object
        mesh = trimesh.Trimesh(vertices=points, faces=faces)

        # Generate grid on elastomer
        elastomer_dims = np.diff(mesh_bounds, axis=0).squeeze()
        slim_axis = np.argmin(elastomer_dims)  # Determine flat axis of elastomer

        # Determine tip direction using dome geometry
        # For dome-shaped elastomers, the center of mass is shifted toward the dome (contact) side
        mesh_center_of_mass = mesh.center_mass[slim_axis]
        bounding_box_center = (mesh_bounds[0, slim_axis] + mesh_bounds[1, slim_axis]) / 2.0

        tip_direction_sign = 1.0 if mesh_center_of_mass > bounding_box_center else -1.0
        if getattr(self.cfg, "flip_tactile_axis", False):
            tip_direction_sign *= -1.0

        # Use the directly specified distance between adjacent tactile points
        tactile_points_dx = tactile_points_distance

        # Sample points on the flat plane
        planar_grid_points = []
        center = (mesh_bounds[0] + mesh_bounds[1]) / 2.0
        idx = 0
        for axis_i in range(3):
            if axis_i == slim_axis:
                # On the slim axis, place a point far away so ray is pointing at the elastomer tip
                planar_grid_points.append([tip_direction_sign])
            else:
                axis_grid_points = np.linspace(
                    center[axis_i] - tactile_points_dx * (num_divs[idx] + 1.0) / 2.0,
                    center[axis_i] + tactile_points_dx * (num_divs[idx] + 1.0) / 2.0,
                    num_divs[idx] + 2,
                )
                planar_grid_points.append(axis_grid_points[1:-1])  # Leave out the extreme corners
                idx += 1

        grid_corners = itertools.product(planar_grid_points[0], planar_grid_points[1], planar_grid_points[2])
        grid_corners = np.array(list(grid_corners))

        # Project ray in positive y direction on the mesh
        mesh_data = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
        ray_dir = np.array([0, 0, 0])
        ray_dir[slim_axis] = -tip_direction_sign  # Ray points towards elastomer (opposite of tip direction)

        # Handle the ray intersection result
        index_tri, index_ray, locations = mesh_data.intersects_id(
            grid_corners, np.tile([ray_dir], (grid_corners.shape[0], 1)), return_locations=True, multiple_hits=False
        )

        if visualize:
            query_pointcloud = trimesh.PointCloud(locations, colors=(0.0, 0.0, 1.0))
            trimesh.Scene([mesh, query_pointcloud]).show()

        expected_num_pts = self.cfg.tactile_array_size[0] * self.cfg.tactile_array_size[1]
        self.expected_num_tactile_points = expected_num_pts
        self.num_valid_tactile_points = int(index_ray.shape[0])

        # Store tactile points in fixed slots so downstream tensors keep a stable shape.
        tactile_points = torch.zeros((expected_num_pts, 3), dtype=torch.float32, device=self._device)
        valid_tactile_mask = torch.zeros(expected_num_pts, dtype=torch.bool, device=self._device)
        if self.num_valid_tactile_points > 0:
            hit_indices = torch.as_tensor(index_ray, dtype=torch.long, device=self._device)
            hit_locations = torch.as_tensor(locations, dtype=torch.float32, device=self._device)
            tactile_points[hit_indices] = hit_locations
            valid_tactile_mask[hit_indices] = True

        self._tactile_pos_local = tactile_points
        self._valid_tactile_mask_local = valid_tactile_mask
        self.num_tactile_points = expected_num_pts

        if self.num_valid_tactile_points != expected_num_pts:
            warning_msg = (
                f"Generated {self.num_valid_tactile_points}/{expected_num_pts} tactile points "
                f"for '{elastomer_prim_path}'; invalid slots will be zero-padded."
            )
            logger.warning(warning_msg)
            print(f"[WARN] {warning_msg}")

        # Assume tactile frame rotation are all the same
        rotation = torch.tensor([0, 0, -torch.pi], device=self._device)
        self._tactile_quat_local = (
            math_utils.quat_from_euler_xyz(rotation[0], rotation[1], rotation[2])
            .unsqueeze(0)
            .repeat(expected_num_pts, 1)
        )

        # Keep the penetration depth convention unchanged (positive scalar depth),
        # but align the projected normal-force axis with the sampled tactile face.
        self._tactile_normal_axis = (
            torch.eye(3, dtype=torch.float32, device=self._device)[slim_axis] * tip_direction_sign
        )

        logger.info(
            "Generated %d valid tactile points from USD mesh using ray casting (template size: %d)",
            self.num_valid_tactile_points,
            expected_num_pts,
        )

    def _initialize_force_field_buffers(self):
        """Initialize data buffers for force field sensing."""
        num_pts = self.num_tactile_points

        # Initialize force field data tensors
        self._data.tactile_points_pos_w = torch.zeros((self._num_envs, num_pts, 3), device=self._device)
        self._data.tactile_points_quat_w = torch.zeros((self._num_envs, num_pts, 4), device=self._device)
        self._data.tactile_valid_mask = self._valid_tactile_mask_local.unsqueeze(0).expand(self._num_envs, -1).clone()
        self._data.num_valid_tactile_points = torch.full(
            (self._num_envs,), self.num_valid_tactile_points, dtype=torch.long, device=self._device
        )
        self._data.penetration_depth = torch.zeros((self._num_envs, num_pts), device=self._device)
        self._data.tactile_normal_force = torch.zeros((self._num_envs, num_pts), device=self._device)
        self._data.tactile_shear_force = torch.zeros((self._num_envs, num_pts, 2), device=self._device)
        # Pre-compute expanded tactile point tensors to avoid repeated unsqueeze/expand operations
        self._tactile_pos_expanded = self._tactile_pos_local.unsqueeze(0).expand(self._num_envs, -1, -1)
        self._tactile_quat_expanded = self._tactile_quat_local.unsqueeze(0).expand(self._num_envs, -1, -1)

    def _initialize_visualization(self):
        """Initialize visualization markers for tactile points."""
        if self.cfg.visualizer_cfg and self._tactile_visualizer is None:
            self._tactile_visualizer = VisualizationMarkers(self.cfg.visualizer_cfg)

    def _update_buffers_impl(self, env_ids: Sequence[int]):
        """Fills the buffers of the sensor data.

        This method updates force-field tactile data for the specified environments.

        Args:
            env_ids: Sequence of environment indices to update. If length equals
                    total number of environments, all environments are updated.
        """
        # Convert to proper indices for internal methods
        if len(env_ids) == self._num_envs:
            internal_env_ids = slice(None)
        else:
            internal_env_ids = env_ids

        # Update force field tactile data
        if self.cfg.enable_force_field:
            self._update_force_field(internal_env_ids)

    #########################################################################################
    # Force field tactile sensing
    #########################################################################################

    def _update_force_field(self, env_ids: Sequence[int] | slice):
        """Update force field tactile sensing data.

        This method computes penalty-based tactile forces using Signed Distance Field (SDF)
        queries. It transforms tactile points to contact object local coordinates, queries the SDF of the
        contact object for collision detection, and computes normal and shear forces based on
        penetration depth and relative velocities.

        Args:
            env_ids: Environment indices or slice to update.

        Note:
            Requires both elastomer and contact object body views to be initialized. Returns
            early if tactile points or body views are not available.
        """
        # Step 1: Get elastomer pose and precompute pose components
        elastomer_pos_w, elastomer_quat_w = self._elastomer_body_view.get_transforms().split([3, 4], dim=-1)
        elastomer_quat_w = math_utils.convert_quat(elastomer_quat_w, to="wxyz")

        # Transform tactile points to world coordinates, used for visualization
        self._transform_tactile_points_to_world(elastomer_pos_w, elastomer_quat_w)

        # earlly return if contact object body view is not available
        # this could happen if the contact object is not specified when tactile_points are required for visualization
        if self._contact_object_body_view is None:
            return

        # Step 2: Transform tactile points to contact object local frame for SDF queries
        contact_object_pos_w, contact_object_quat_w = self._contact_object_body_view.get_transforms().split(
            [3, 4], dim=-1
        )
        contact_object_quat_w = math_utils.convert_quat(contact_object_quat_w, to="wxyz")

        world_tactile_points = self._data.tactile_points_pos_w
        contact_object_scale = self._get_contact_object_scales()
        points_contact_object_local, contact_object_quat_inv = self._transform_points_to_contact_object_local(
            world_tactile_points, contact_object_pos_w, contact_object_quat_w, contact_object_scale
        )

        # Step 3: Query SDF for collision detection
        sdf_values_and_gradients = self._contact_object_sdf_view.get_sdf_and_gradients(points_contact_object_local)
        sdf_values = sdf_values_and_gradients[..., -1]  # Last component is SDF value
        sdf_gradients = sdf_values_and_gradients[..., :-1]  # First 3 components are gradients

        # Step 4: Compute tactile forces from SDF data
        self._compute_tactile_forces_from_sdf(
            points_contact_object_local,
            sdf_values,
            sdf_gradients,
            contact_object_pos_w,
            contact_object_quat_w,
            contact_object_scale,
            elastomer_quat_w,
            env_ids,
        )

    def _transform_tactile_points_to_world(self, pos_w: torch.Tensor, quat_w: torch.Tensor):
        """Transform tactile points from local to world coordinates.

        Args:
            pos_w: Elastomer positions in world frame. Shape: (num_envs, 3)
            quat_w: Elastomer quaternions in world frame. Shape: (num_envs, 4)
        """
        num_pts = self.num_tactile_points

        quat_expanded = quat_w.unsqueeze(1).expand(-1, num_pts, -1)
        pos_expanded = pos_w.unsqueeze(1).expand(-1, num_pts, -1)

        # Apply transformation
        tactile_pos_w = math_utils.quat_apply(quat_expanded, self._tactile_pos_expanded) + pos_expanded
        tactile_quat_w = math_utils.quat_mul(quat_expanded, self._tactile_quat_expanded)

        # Store in data
        self._data.tactile_points_pos_w = tactile_pos_w
        self._data.tactile_points_quat_w = tactile_quat_w
        invalid_mask = ~self._data.tactile_valid_mask
        self._data.tactile_points_pos_w[invalid_mask] = 0.0

    def _transform_points_to_contact_object_local(
        self,
        world_points: torch.Tensor,
        contact_object_pos_w: torch.Tensor,
        contact_object_quat_w: torch.Tensor,
        contact_object_scale: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Optimized version: Transform world coordinates to contact object local frame.

        Args:
            world_points: Points in world coordinates. Shape: (num_envs, num_points, 3)
            contact_object_pos_w: Contact object positions in world frame. Shape: (num_envs, 3)
            contact_object_quat_w: Contact object quaternions in world frame. Shape: (num_envs, 4)

        Returns:
            Points in contact object local coordinates and inverse quaternions
        """
        # Get inverse transformation (per environment)
        # wxyz in torch
        contact_object_quat_inv, _ = torch_utils.tf_inverse(contact_object_quat_w, contact_object_pos_w)
        num_pts = self.num_tactile_points

        contact_object_quat_expanded = contact_object_quat_inv.unsqueeze(1).expand(-1, num_pts, 4)
        points_object_local = math_utils.quat_apply(
            contact_object_quat_expanded,
            world_points - contact_object_pos_w.unsqueeze(1),
        )
        points_sdf = points_object_local / contact_object_scale.unsqueeze(1).clamp(min=1e-8)

        return points_sdf, contact_object_quat_inv

    def _get_tactile_points_velocities(
        self, linvel_world: torch.Tensor, angvel_world: torch.Tensor, quat_world: torch.Tensor
    ) -> torch.Tensor:
        """Optimized version: Compute tactile point velocities from precomputed velocities.

        Args:
            linvel_world: Elastomer linear velocities. Shape: (num_envs, 3)
            angvel_world: Elastomer angular velocities. Shape: (num_envs, 3)
            quat_world: Elastomer quaternions. Shape: (num_envs, 4)

        Returns:
            Tactile point velocities in world frame. Shape: (num_envs, num_points, 3)
        """
        num_pts = self.num_tactile_points

        # Pre-expand all required tensors once
        quat_expanded = quat_world.unsqueeze(1).expand(-1, num_pts, 4)
        tactile_pos_expanded = self._tactile_pos_local.unsqueeze(0).expand(quat_world.shape[0], -1, -1)

        # Transform local positions to world frame relative vectors
        tactile_pos_world_relative = math_utils.quat_apply(quat_expanded, tactile_pos_expanded)

        # Compute velocity due to angular motion: ω × r
        angvel_expanded = angvel_world.unsqueeze(1).expand(-1, num_pts, 3)
        angular_velocity_contribution = torch.cross(angvel_expanded, tactile_pos_world_relative, dim=-1)

        # Add linear velocity contribution
        linvel_expanded = linvel_world.unsqueeze(1).expand(-1, num_pts, 3)
        tactile_velocity_world = angular_velocity_contribution + linvel_expanded

        return tactile_velocity_world

    def _compute_tactile_forces_from_sdf(
        self,
        points_contact_object_local: torch.Tensor,
        sdf_values: torch.Tensor,
        sdf_gradients: torch.Tensor,
        contact_object_pos_w: torch.Tensor,
        contact_object_quat_w: torch.Tensor,
        contact_object_scale: torch.Tensor,
        elastomer_quat_w: torch.Tensor,
        env_ids: Sequence[int] | slice,
    ) -> None:
        """Optimized version: Compute tactile forces from SDF values using precomputed parameters.

        This method now operates directly on the pre-allocated data tensors to avoid
        unnecessary memory allocation and copying.

        Args:
            points_contact_object_local: Points in contact object local frame
            sdf_values: SDF values (negative means penetration)
            sdf_gradients: SDF gradients (surface normals)
            contact_object_pos_w: Contact object positions in world frame
            contact_object_quat_w: Contact object quaternions in world frame
            elastomer_quat_w: Elastomer quaternions
            env_ids: Environment indices being updated

        """
        depth = self._data.penetration_depth[env_ids]
        tactile_normal_force = self._data.tactile_normal_force[env_ids]
        tactile_shear_force = self._data.tactile_shear_force[env_ids]

        # Clear the output tensors
        tactile_normal_force.zero_()
        tactile_shear_force.zero_()
        depth.zero_()

        valid_mask = self._data.tactile_valid_mask[env_ids]
        contact_object_scale_env = contact_object_scale[env_ids]
        scale_expanded = contact_object_scale_env.unsqueeze(1).expand(-1, self.num_tactile_points, -1)
        local_normals = torch.nn.functional.normalize(sdf_gradients[env_ids], dim=-1)
        depth_scale = torch.linalg.norm(local_normals * scale_expanded, dim=-1).clamp(min=1e-8)

        # Convert SDF values to penetration depth in world units (positive for penetration)
        depth[:] = torch.clamp(-sdf_values[env_ids], min=0.0) * depth_scale
        depth[~valid_mask] = 0.0

        # Get collision mask for points that are penetrating
        collision_mask = valid_mask & (depth > 0.0)

        # Use pre-allocated tensors instead of creating new ones
        num_pts = self.num_tactile_points

        if collision_mask.any() or self.cfg.visualize_sdf_closest_pts:
            # Get contact object and elastomer velocities (com velocities)
            contact_object_velocities = self._contact_object_body_view.get_velocities()
            contact_object_linvel_w_com = contact_object_velocities[env_ids, :3]
            contact_object_angvel_w = contact_object_velocities[env_ids, 3:]

            elastomer_velocities = self._elastomer_body_view.get_velocities()
            elastomer_linvel_w_com = elastomer_velocities[env_ids, :3]
            elastomer_angvel_w = elastomer_velocities[env_ids, 3:]

            # Contact object adjustment
            contact_object_com_w_offset = math_utils.quat_apply(
                contact_object_quat_w[env_ids], self._contact_object_com_b[env_ids]
            )
            contact_object_linvel_w = contact_object_linvel_w_com - torch.cross(
                contact_object_angvel_w, contact_object_com_w_offset, dim=-1
            )
            # v_origin = v_com - w x (com_world_offset) where com_world_offset = quat_apply(quat, com_b)
            elastomer_com_w_offset = math_utils.quat_apply(elastomer_quat_w[env_ids], self._elastomer_com_b[env_ids])
            elastomer_linvel_w = elastomer_linvel_w_com - torch.cross(
                elastomer_angvel_w, elastomer_com_w_offset, dim=-1
            )

            normals_scaled_local = torch.nn.functional.normalize(local_normals / scale_expanded.clamp(min=1e-8), dim=-1)

            # Transform normals to world frame (rotate by contact object orientation) - use precomputed quaternions
            contact_object_quat_expanded = contact_object_quat_w[env_ids].unsqueeze(1).expand(-1, num_pts, 4)

            # Apply quaternion transformation
            normals_world = math_utils.quat_apply(contact_object_quat_expanded, normals_scaled_local)

            # Compute normal contact force: F_n = k_n * depth
            fc_norm = self.cfg.normal_contact_stiffness * depth
            fc_world = fc_norm.unsqueeze(-1) * normals_world

            # Get tactile point velocities using precomputed velocities
            tactile_velocity_world = self._get_tactile_points_velocities(
                elastomer_linvel_w, elastomer_angvel_w, elastomer_quat_w[env_ids]
            )

            # Use precomputed contact object velocities
            depth_local = depth / depth_scale
            closest_points_sdf = points_contact_object_local[env_ids] + depth_local.unsqueeze(-1) * local_normals

            if self.cfg.visualize_sdf_closest_pts:
                debug_closest_points_sdf = (
                    points_contact_object_local[env_ids] - sdf_values[env_ids].unsqueeze(-1) * local_normals
                )
                self.debug_closest_points_wolrd = math_utils.quat_apply(
                    contact_object_quat_expanded, debug_closest_points_sdf * scale_expanded
                ) + contact_object_pos_w[env_ids].unsqueeze(1).expand(-1, num_pts, 3)
                self.debug_closest_points_wolrd[~valid_mask] = 0.0

            contact_object_linvel_expanded = contact_object_linvel_w.unsqueeze(1).expand(-1, num_pts, 3)
            contact_object_angvel_expanded = contact_object_angvel_w.unsqueeze(1).expand(-1, num_pts, 3)
            closest_points_vel_world = (
                torch.linalg.cross(
                    contact_object_angvel_expanded,
                    math_utils.quat_apply(contact_object_quat_expanded, closest_points_sdf * scale_expanded),
                )
                + contact_object_linvel_expanded
            )

            # Compute relative velocity at contact points
            relative_velocity_world = tactile_velocity_world - closest_points_vel_world

            # Compute tangential velocity (perpendicular to normal)
            vt_world = relative_velocity_world - normals_world * torch.sum(
                normals_world * relative_velocity_world, dim=-1, keepdim=True
            )
            vt_norm = torch.norm(vt_world, dim=-1)

            # Compute friction force: F_t = min(k_t * |v_t|, mu * F_n)
            ft_static_norm = self.cfg.tangential_stiffness * vt_norm
            ft_dynamic_norm = self.cfg.friction_coefficient * fc_norm
            ft_norm = torch.minimum(ft_static_norm, ft_dynamic_norm)

            # Apply friction force opposite to tangential velocity
            ft_world = -ft_norm.unsqueeze(-1) * vt_world / (vt_norm.unsqueeze(-1).clamp(min=1e-9))

            # Total tactile force in world frame
            tactile_force_world = fc_world + ft_world

            # Transform forces to tactile frame
            tactile_force_tactile = math_utils.quat_apply_inverse(
                self._data.tactile_points_quat_w[env_ids], tactile_force_world
            )

            # Extract normal and shear components in the generated taxel frame.
            tactile_normal_force[:] = torch.sum(tactile_force_tactile * self._tactile_normal_axis.view(1, 1, 3), dim=-1)
            tactile_shear_force[:] = tactile_force_tactile[..., :2]

        tactile_normal_force[~valid_mask] = 0.0
        tactile_shear_force[~valid_mask] = 0.0
        depth[~valid_mask] = 0.0

    #########################################################################################
    # Debug visualization
    #########################################################################################

    def _set_debug_vis_impl(self, debug_vis: bool):
        """Set debug visualization into visualization objects."""
        # set visibility of markers
        # note: parent only deals with callbacks. not their visibility
        if debug_vis:
            # create markers if necessary for the first time
            if self._tactile_visualizer is None:
                self._tactile_visualizer = VisualizationMarkers(self.cfg.visualizer_cfg)
            # set their visibility to true
            self._tactile_visualizer.set_visibility(True)
        else:
            if self._tactile_visualizer:
                self._tactile_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        """Callback for debug visualization of tactile sensor data.

        This method is called during each simulation step when debug visualization is enabled.
        It visualizes tactile sensing points as 3D markers in the simulation viewport to help
        with debugging and understanding sensor behavior.

        The method handles two visualization modes:

        1. **Standard mode**: Visualizes ``tactile_points_pos_w`` - the world positions of
            tactile sensing points on the sensor surface
        2. **SDF debug mode**: When ``cfg.visualize_sdf_closest_pts`` is True, visualizes
            ``debug_closest_points_wolrd`` - the closest surface points computed during
            SDF-based force calculations
        """
        # Safety check - return if not properly initialized
        if not hasattr(self, "_tactile_visualizer") or self._tactile_visualizer is None:
            return
        vis_points = None

        if self.cfg.visualize_sdf_closest_pts and hasattr(self, "debug_closest_points_wolrd"):
            vis_points = self.debug_closest_points_wolrd
        else:
            vis_points = self._data.tactile_points_pos_w

        if vis_points is None or vis_points.numel() == 0:
            return

        valid_mask = self._data.tactile_valid_mask
        if valid_mask is not None:
            viz_points = vis_points[valid_mask]
        else:
            viz_points = vis_points.view(-1, 3)  # Shape: (num_envs * num_points, 3)

        if viz_points.numel() == 0:
            return

        indices = torch.zeros(viz_points.shape[0], dtype=torch.long, device=self._device)

        marker_scales = torch.ones(viz_points.shape[0], 3, device=self._device)

        # Visualize tactile points
        self._tactile_visualizer.visualize(translations=viz_points, marker_indices=indices, scales=marker_scales)
