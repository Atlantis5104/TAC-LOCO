# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the tactile-array sensor."""

from __future__ import annotations

from dataclasses import MISSING

from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.markers.config import VISUO_TACTILE_SENSOR_MARKER_CFG
from isaaclab.sensors import SensorBaseCfg
from isaaclab.utils import configclass

from .tactile_array_sensor import TactileArraySensor


@configclass
class TactileArraySensorCfg(SensorBaseCfg):
    """Configuration for fixed-shape SDF-based tactile-array sensing."""

    class_type: type = TactileArraySensor

    enable_force_field: bool = True
    """Enable SDF-based contact depth and force computation."""

    tactile_array_size: tuple[int, int] = MISSING
    """Taxel grid size as ``(rows, columns)``."""

    tactile_points_distance: float = MISSING
    """Distance between adjacent taxels in metres."""

    flip_tactile_axis: bool = False
    """Flip the tactile surface direction inferred from the elastomer mesh."""

    contact_object_prim_path_expr: str | None = None
    """Contact-object prim expression; ``{ENV_REGEX_NS}`` is supported internally."""

    normal_contact_stiffness: float = 1.0
    """Normal penalty stiffness."""

    friction_coefficient: float = 2.0
    """Coulomb friction coefficient used to cap shear force."""

    tangential_stiffness: float = 0.1
    """Tangential velocity penalty stiffness."""

    visualizer_cfg: VisualizationMarkersCfg = VISUO_TACTILE_SENSOR_MARKER_CFG.replace(
        prim_path="/Visuals/TactileArraySensor"
    )
    """Marker configuration used when ``debug_vis`` is enabled."""

    trimesh_vis_tactile_points: bool = False
    """Open a trimesh window for taxel-generation debugging."""

    visualize_sdf_closest_pts: bool = False
    """Visualize closest SDF surface points instead of taxel positions."""
