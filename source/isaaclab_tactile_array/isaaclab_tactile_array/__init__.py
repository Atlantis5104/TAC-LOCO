# Copyright (c) 2026, Tactile Array Sensor contributors.
# SPDX-License-Identifier: BSD-3-Clause

"""Standalone tactile-array sensing for Isaac Lab 2.3.2."""

from pathlib import Path


__version__ = "0.1.0"

_PACKAGE_DIR = Path(__file__).resolve().parent
ISAACLAB_TACTILE_ARRAY_ASSETS_DIR = str(_PACKAGE_DIR / "assets")
TACTILE_ARRAY_FINGER_USD_PATH = str(_PACKAGE_DIR / "assets" / "tactile_array_finger.usd")


def get_asset_path(relative_path: str) -> str:
    """Return an absolute path to a packaged asset.

    Args:
        relative_path: Path relative to the package's ``assets`` directory.

    Raises:
        ValueError: If ``relative_path`` escapes the assets directory.
        FileNotFoundError: If the requested asset is not installed.
    """
    assets_dir = Path(ISAACLAB_TACTILE_ARRAY_ASSETS_DIR).resolve()
    asset_path = (assets_dir / relative_path).resolve()
    try:
        asset_path.relative_to(assets_dir)
    except ValueError as exc:
        raise ValueError(f"Asset path must stay inside {assets_dir}: {relative_path!r}") from exc
    if not asset_path.is_file():
        raise FileNotFoundError(f"Packaged tactile-array asset not found: {asset_path}")
    return str(asset_path)


from .sensors import TactileArraySensor, TactileArraySensorCfg, TactileArraySensorData  # noqa: E402

__all__ = [
    "__version__",
    "ISAACLAB_TACTILE_ARRAY_ASSETS_DIR",
    "TACTILE_ARRAY_FINGER_USD_PATH",
    "TactileArraySensor",
    "TactileArraySensorCfg",
    "TactileArraySensorData",
    "get_asset_path",
]
