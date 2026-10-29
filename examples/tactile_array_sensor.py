# Copyright (c) 2026, Tactile Array Sensor contributors.
# SPDX-License-Identifier: BSD-3-Clause

"""Minimal tactile-array demo for an unmodified Isaac Lab v2.3.2 checkout.

Run it with the Python selected by Isaac Lab::

    /path/to/IsaacLab/isaaclab.sh -p examples/tactile_array_sensor.py

For a short, headless validation run::

    /path/to/IsaacLab/isaaclab.sh -p examples/tactile_array_sensor.py \
        --headless --max_steps 100
"""

from __future__ import annotations

import argparse
import math
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Run the standalone tactile-array sensor demo.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments.")
parser.add_argument("--tactile_rows", type=int, default=12, help="Number of taxel rows.")
parser.add_argument("--tactile_cols", type=int, default=32, help="Number of taxel columns.")
parser.add_argument("--tactile_spacing", type=float, default=0.002, help="Taxel spacing in metres.")
parser.add_argument(
    "--tactile_compliance_stiffness",
    type=float,
    default=None,
    help="Override compliant contact stiffness (default: use the USD asset value).",
)
parser.add_argument(
    "--tactile_compliant_damping",
    type=float,
    default=None,
    help="Override compliant contact damping (default: use the USD asset value).",
)
parser.add_argument("--debug_sdf_closest_pts", action="store_true", help="Visualize closest SDF points.")
parser.add_argument("--debug_tactile_sensor_pts", action="store_true", help="Visualize tactile sensor points.")
parser.add_argument("--trimesh_vis_tactile_points", action="store_true", help="Visualize tactile points using trimesh.")
parser.add_argument(
    "--contact_object_scale",
    type=float,
    nargs=3,
    default=(1.0, 1.0, 1.0),
    metavar=("X", "Y", "Z"),
    help="Non-uniform scale for the SDF contact object.",
)
parser.add_argument(
    "--max_steps",
    type=int,
    default=None,
    help="Exit after this many physics steps and validate all output tensors.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Isaac Sim and Isaac Lab modules must be imported after AppLauncher starts the app.
import numpy as np  # noqa: E402
import torch  # noqa: E402
from isaaclab_tactile_array import (  # noqa: E402
    TACTILE_ARRAY_FINGER_USD_PATH,
    TactileArraySensorCfg,
    TactileArraySensorData,
)

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # noqa: E402


def _print_status(message: str) -> None:
    """Write through Kit's stdout redirection so smoke-test results stay visible."""

    print(message, file=sys.__stdout__, flush=True)


@configclass
class TactileArraySceneCfg(InteractiveSceneCfg):
    """One tactile finger and one SDF-enabled contact object."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileWithCompliantContactCfg(
            usd_path=TACTILE_ARRAY_FINGER_USD_PATH,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            compliant_contact_stiffness=args_cli.tactile_compliance_stiffness,
            compliant_contact_damping=args_cli.tactile_compliant_damping,
            physics_material_prim_path="elastomer",
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.002,
                rest_offset=-0.0006,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.5),
            rot=(math.sqrt(2.0) / 2.0, -math.sqrt(2.0) / 2.0, 0.0, 0.0),
            joint_pos={},
            joint_vel={},
        ),
        actuators={},
    )

    contact_object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/contact_object",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Factory/factory_peg_8mm.usd",
            scale=tuple(args_cli.contact_object_scale),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
                max_angular_velocity=180.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.5),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.005,
                rest_offset=-0.0006,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(articulation_enabled=False),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.05, 0.023, 0.65),
            rot=(0.70710678, 0.70710678, 0.0, 0.0),
        ),
    )

    tactile_sensor = TactileArraySensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/elastomer/tactile_sensor",
        debug_vis=args_cli.debug_tactile_sensor_pts or args_cli.debug_sdf_closest_pts,
        tactile_array_size=(args_cli.tactile_rows, args_cli.tactile_cols),
        tactile_points_distance=args_cli.tactile_spacing,
        contact_object_prim_path_expr="{ENV_REGEX_NS}/contact_object",
        enable_force_field=True,
        normal_contact_stiffness=1.0,
        friction_coefficient=2.0,
        tangential_stiffness=0.1,
        trimesh_vis_tactile_points=args_cli.trimesh_vis_tactile_points,
        visualize_sdf_closest_pts=args_cli.debug_sdf_closest_pts,
    )


def visualize_tactile_heatmap(
    data: TactileArraySensorData, nrows: int, ncols: int, env_idx: int = 0, scale_factor: int = 20
) -> None:
    """Show the original pixelated Viridis heatmap with per-frame adaptive scaling."""

    import cv2

    force_grid = np.abs(data.tactile_normal_force[env_idx].detach().cpu().numpy()).reshape(nrows, ncols)
    max_value = float(force_grid.max())
    denominator = 0.0008 if max_value < 0.0002 else max_value
    normalized = np.clip(force_grid / denominator, 0.0, 1.0)
    pixels = (normalized * 255).astype(np.uint8)
    pixels = np.repeat(np.repeat(pixels, scale_factor, axis=0), scale_factor, axis=1)
    image = cv2.applyColorMap(pixels, cv2.COLORMAP_VIRIDIS)
    cv2.imshow("Tactile Normal Force Heatmap", image)
    cv2.waitKey(1)


def validate_tactile_data(data: TactileArraySensorData, num_envs: int, num_points: int) -> None:
    """Check the public fixed-shape output contract used by downstream tasks."""

    expected_shapes = {
        "tactile_points_pos_w": (num_envs, num_points, 3),
        "tactile_points_quat_w": (num_envs, num_points, 4),
        "tactile_valid_mask": (num_envs, num_points),
        "num_valid_tactile_points": (num_envs,),
        "penetration_depth": (num_envs, num_points),
        "tactile_normal_force": (num_envs, num_points),
        "tactile_shear_force": (num_envs, num_points, 2),
    }
    for field_name, expected_shape in expected_shapes.items():
        value = getattr(data, field_name)
        if value is None:
            raise RuntimeError(f"Tactile output {field_name!r} was not initialized.")
        if tuple(value.shape) != expected_shape:
            raise RuntimeError(
                f"Tactile output {field_name!r} has shape {tuple(value.shape)}; expected {expected_shape}."
            )
        if value.is_floating_point() and not torch.isfinite(value).all():
            raise RuntimeError(f"Tactile output {field_name!r} contains non-finite values.")


def run_simulation(sim: sim_utils.SimulationContext, scene: InteractiveScene) -> None:
    """Step physics, report contact, and optionally perform a bounded smoke test."""

    sim_dt = sim.get_physics_dt()
    force = torch.zeros(scene.num_envs, 1, 3, device=sim.device)
    torque = torch.zeros_like(force)
    env_ids = torch.arange(scene.num_envs, device=sim.device)
    torque[:, 0, 1] = torch.where(env_ids % 2 == 0, -20.0, 20.0)

    num_points = args_cli.tactile_rows * args_cli.tactile_cols
    step = 0
    cycle_step = 0
    saw_contact = False
    while simulation_app.is_running():
        if cycle_step > 20:
            scene["contact_object"].set_external_force_and_torque(force, torque)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        step += 1
        cycle_step += 1

        data = scene["tactile_sensor"].data
        if not args_cli.headless and step % 10 == 0:
            visualize_tactile_heatmap(data, args_cli.tactile_rows, args_cli.tactile_cols)
        saw_contact = saw_contact or bool(torch.any(data.tactile_normal_force > 0.0).item())
        if step % 25 == 0:
            active = (data.tactile_normal_force > 0.0).sum(dim=1).tolist()
            peak = data.tactile_normal_force.amax(dim=1).tolist()
            _print_status(f"[INFO] step={step:04d} active_taxels={active} peak_normal_force={peak}")

        if args_cli.max_steps is not None and step >= args_cli.max_steps:
            validate_tactile_data(data, scene.num_envs, num_points)
            if not saw_contact:
                raise RuntimeError(
                    "No tactile contact was observed during the bounded run; increase --max_steps "
                    "or inspect the demo scene."
                )
            valid = data.num_valid_tactile_points.tolist()
            _print_status(f"[INFO] Tactile-array validation passed after {step} steps; valid taxels={valid}.")
            return

        if cycle_step == 200:
            for entity_name in ("robot", "contact_object"):
                root_state = scene[entity_name].data.default_root_state.clone()
                root_state[:, :3] += scene.env_origins
                scene[entity_name].write_root_state_to_sim(root_state)
            scene.reset()
            cycle_step = 0


def main() -> None:
    """Create the scene and run the tactile-array demo."""

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=0.005,
            device=args_cli.device,
        )
    )
    sim.set_camera_view(eye=(0.12, -0.085, 0.59), target=(0.04, -0.003, 0.51))
    scene = InteractiveScene(TactileArraySceneCfg(num_envs=args_cli.num_envs, env_spacing=0.2))
    _print_status("[INFO] Scene created; resetting simulation...")
    try:
        sim.reset()
    except Exception as error:
        _print_status(f"[ERROR] Simulation reset raised {type(error).__name__}: {error!r}")
        raise
    _print_status("[INFO] Setup complete. Press Ctrl+C to stop.")
    run_simulation(sim, scene)


if __name__ == "__main__":
    try:
        main()
    finally:
        # This is a standalone process; immediate cleanup avoids a v2.3.2 stage-close stall after SDF queries.
        simulation_app.close(skip_cleanup=True)
