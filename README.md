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
../IsaacLab/isaaclab.sh -p examples/tactile_array_sensor.py
```

Use your Isaac Lab path if it differs. Git LFS is needed for the included USD/GIF; the demo downloads its contact-object asset on first use.

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
