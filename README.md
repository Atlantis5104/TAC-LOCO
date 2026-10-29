# Isaac Lab Tactile Array

A tactile-array sensor for official **Isaac Lab v2.3.2**, installed as an independent Python package without modifying Isaac Lab.

![Isaac Lab camera view and tactile-array normal force](docs/media/tactile_array_demo.gif)

## Quick start

Prepare an [Isaac Lab v2.3.2 installation](https://github.com/isaac-sim/IsaacLab/tree/v2.3.2/docs/source/setup/installation) and activate its Python environment. With Isaac Lab in a sibling directory named `IsaacLab`:

```bash
git clone --branch IsaacLab_tac_array --single-branch --depth 1 \
    https://github.com/Atlantis5104/TAC-LOCO.git
cd TAC-LOCO
./install.sh --isaaclab ../IsaacLab
../IsaacLab/isaaclab.sh -p examples/tactile_array_sensor.py --debug_tactile_sensor_pts
```

Use your Isaac Lab path if it differs. Git LFS is needed for the included USD/GIF; the demo downloads its contact-object asset on first use.

The demo also opens the original Viridis normal-force heatmap. Use `--debug_tactile_sensor_pts` to show taxels, `--debug_sdf_closest_pts` to show SDF closest points instead, or `--trimesh_vis_tactile_points` to inspect taxels in a separate trimesh window (close it to continue).

## Parameters

Append these options to the demo command:

| Option | Default | Effect |
| --- | --- | --- |
| `--normal_contact_stiffness` | `1.0` | Normal-force gain applied to penetration depth before projection into the taxel frame |
| `--tangential_stiffness` | `0.1` | Tangential-velocity gain for estimating shear force |
| `--friction_coefficient` | `2.0` | Coulomb cap on estimated shear force (`mu × normal force`) |
| `--tactile_compliance_stiffness` | USD asset value | PhysX compliant-contact stiffness; lower values allow softer contact |
| `--tactile_compliant_damping` | USD asset value | PhysX compliant-contact damping |
| `--tactile_rows`, `--tactile_cols` | `12`, `32` | Number of taxel rows and columns |
| `--tactile_spacing` | `0.002` m | Distance between neighboring taxels |
| `--contact_object_scale X Y Z` | `1 1 1` | Contact-object scale along each axis |
| `--num_envs` | `1` | Number of parallel environments |

The first three options change sensor force estimates; the compliance options change simulated contact dynamics. `--friction_coefficient` does not change PhysX material friction. Heatmap colors are normalized per frame, so compare raw force observations when tuning gains.

In Python, the first three are `TactileArraySensorCfg` fields; compliance is configured on the asset spawner through `compliant_contact_stiffness` and `compliant_contact_damping`.

For example:

```bash
../IsaacLab/isaaclab.sh -p examples/tactile_array_sensor.py \
    --debug_tactile_sensor_pts --normal_contact_stiffness 2.0 \
    --friction_coefficient 1.0 --tangential_stiffness 0.2 \
    --tactile_compliance_stiffness 30.0 --tactile_compliant_damping 0.1
```

## Observations

Import `TactileArraySensorCfg` from `isaaclab_tactile_array` after starting `AppLauncher`. Add it to your scene, then read `scene["tactile_sensor"].data`:

| Field | Shape | Description |
| --- | --- | --- |
| `tactile_normal_force` | `(N, P)` | Normal force at each taxel |
| `tactile_shear_force` | `(N, P, 2)` | Two tangential force components in the taxel frame |
| `penetration_depth` | `(N, P)` | Contact penetration depth in metres |
| `tactile_points_pos_w` | `(N, P, 3)` | Taxel positions in world coordinates |
| `tactile_points_quat_w` | `(N, P, 4)` | Taxel orientations in world coordinates (`wxyz`) |
| `tactile_valid_mask` | `(N, P)` | Whether each taxel intersects the sensor surface |
| `num_valid_tactile_points` | `(N,)` | Number of valid taxels per environment |

`N` is the number of environments; `P = rows × columns` (the demo uses `12 × 32 = 384`). Invalid slots remain in the tensors with zero position, depth, and force values. Forces use a penalty model with configurable stiffness and friction; enable them with `enable_force_field=True`, as in the demo.
