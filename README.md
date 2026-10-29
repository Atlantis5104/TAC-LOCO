# Isaac Lab Tactile Array

A standalone, fixed-shape tactile-array sensor for an **unmodified official
[Isaac Lab v2.3.2](https://github.com/isaac-sim/IsaacLab/tree/v2.3.2)** checkout.
It reports taxel positions, penetration, normal force, and 2-D shear force from GPU SDF queries.

中文概要：用户只需准备官方 Isaac Lab `v2.3.2`，再运行本仓库的 `install.sh`；安装过程不会修改官方源码。

![Tactile-array normal and shear force demo](docs/media/tactile_array_demo.gif)

The animation above was generated from this sensor's simulation output: a `12 × 32` normal-force heatmap on the left and normal/shear vectors on the right. It does not use HydroShear data.

## What is included

- The stable SDF implementation used by the Go2 tactile task, packaged as `isaaclab-tactile-array`.
- A fixed `(num_envs, rows × columns, ...)` tensor contract with a validity mask for missing ray hits.
- Normal force, two-axis shear force, penetration depth, taxel world poses, and validity counts.
- Correct SDF queries for uniformly or non-uniformly scaled contact objects.
- A self-contained tactile-finger USD asset and a runnable example.
- One installer that validates the exact Isaac Lab revision and installs the add-on into Isaac Lab's Python environment.

HydroShear, FFT processing, camera rendering, and the experimental batched tactile manager are intentionally outside this release.

## Compatibility

| Component | Supported version |
| --- | --- |
| Isaac Lab | `v2.3.2` (`37ddf626871758333d6ed89cf64ad702aef127d0`) |
| Add-on package | `isaaclab-tactile-array==0.1.0` |
| Platform / Isaac Sim | Whatever is supported by the official Isaac Lab v2.3.2 installation |

The installer rejects a different Isaac Lab commit. This strict pin keeps the public release reproducible instead of silently running against an untested API.

## Quick start

Create two sibling checkouts:

```bash
git clone --branch v2.3.2 --depth 1 https://github.com/isaac-sim/IsaacLab.git
git clone --branch tactile-array-addon --single-branch --depth 1 \
    https://github.com/YuHoChau/IsaacLab_tac_array.git
```

Complete the official
[Isaac Lab v2.3.2 installation](https://github.com/isaac-sim/IsaacLab/tree/v2.3.2/docs/source/setup/installation)
so its `isaaclab.sh` can find Isaac Sim. Activate that environment if the official setup uses Conda or a virtual environment. Then install the add-on:

```bash
cd IsaacLab_tac_array
./install.sh --isaaclab ../IsaacLab
```

`install.sh` uses the Python selected by the official `isaaclab.sh`, ensures that the v2.3.2 core and `isaaclab_contrib` packages point to that checkout, installs the add-on and its packaged asset, and leaves the official checkout unchanged. It is safe to run again.

Run the interactive demo:

```bash
../IsaacLab/isaaclab.sh -p examples/tactile_array_sensor.py
```

Or run a bounded headless smoke test:

```bash
../IsaacLab/isaaclab.sh -p examples/tactile_array_sensor.py \
    --headless --max_steps 100
```

Successful validation ends with a message similar to:

```text
[INFO] Tactile-array validation passed after 100 steps; valid taxels=[384].
```

The example uses the standard Isaac Lab Nucleus peg asset. Its first run therefore needs access to the Isaac asset server configured by the official installation.

### Installer options

```bash
# Verify compatibility without changing the Python environment
./install.sh --check --isaaclab /absolute/path/to/IsaacLab

# These path forms are also accepted
./install.sh /absolute/path/to/IsaacLab
ISAACLAB_PATH=/absolute/path/to/IsaacLab ./install.sh

./install.sh --help
```

The repository uses Git LFS for the USD and README animation. During installation, the script detects an unresolved LFS pointer and fetches only the required release assets. If Git LFS is not available, it prints an actionable error instead of installing a broken asset.

## Python API

Import the sensor after `AppLauncher` has started Isaac Sim:

```python
from isaaclab_tactile_array import TactileArraySensorCfg

tactile_sensor = TactileArraySensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/elastomer/tactile_sensor",
    tactile_array_size=(12, 32),
    tactile_points_distance=0.002,
    contact_object_prim_path_expr="{ENV_REGEX_NS}/contact_object",
    enable_force_field=True,
    normal_contact_stiffness=1.0,
    friction_coefficient=2.0,
    tangential_stiffness=0.1,
)
```

Add this configuration as a field of an `InteractiveSceneCfg`. The contact object must already exist when the simulation starts and must contain a PhysX SDF collision mesh. The add-on resolves `{ENV_REGEX_NS}` internally, so no patch to `InteractiveScene` is required.

The packaged demo asset is available without hard-coded repository paths:

```python
from isaaclab_tactile_array import TACTILE_ARRAY_FINGER_USD_PATH
```

### Output contract

For `N` environments and `P = rows × columns` taxel slots, `scene["tactile_sensor"].data` provides:

| Field | Shape | Meaning |
| --- | --- | --- |
| `tactile_points_pos_w` | `(N, P, 3)` | Taxel positions in world coordinates |
| `tactile_points_quat_w` | `(N, P, 4)` | Taxel orientations in `wxyz` order |
| `tactile_valid_mask` | `(N, P)` | Slots that intersected the elastomer surface |
| `num_valid_tactile_points` | `(N,)` | Valid slot count per environment |
| `penetration_depth` | `(N, P)` | Positive SDF penetration depth |
| `tactile_normal_force` | `(N, P)` | Penalty-model normal force |
| `tactile_shear_force` | `(N, P, 2)` | Penalty-model shear force in the taxel frame |

Invalid taxel slots are retained so tensor shapes never change; their position, penetration, and force values are zeroed.

## Repository layout

```text
install.sh                         one-command installer and compatibility check
examples/tactile_array_sensor.py  runnable example and output-contract smoke test
source/isaaclab_tactile_array/    independently installable Python package
docs/media/                       README demonstration media
```

## Current limitations

- One pre-declared SDF contact object is configured per sensor.
- Contact objects cannot be added or removed dynamically after sensor initialization.
- Taxels are generated from the first non-collision mesh below the elastomer prim.
- The force law is a penalty model; tune its stiffness and friction parameters for the task rather than treating the defaults as calibrated hardware values.

## License and attribution

The add-on source is released under its [BSD-3-Clause license](LICENSE) and is derived from work in the Isaac Lab sensor stack. Isaac Lab, Isaac Sim, Nucleus assets, and other dependencies retain their respective upstream licenses and terms.

This project is an independent add-on and is not an official NVIDIA Isaac Lab release.
