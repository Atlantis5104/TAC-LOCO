# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Output data for the tactile-array sensor."""

from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass
class TactileArraySensorData:
    """Tensors produced by :class:`TactileArraySensor`."""

    tactile_points_pos_w: torch.Tensor | None = None
    """Taxel positions in world frame, shaped ``(N, P, 3)``."""

    tactile_points_quat_w: torch.Tensor | None = None
    """Taxel orientations in world frame (wxyz), shaped ``(N, P, 4)``."""

    tactile_valid_mask: torch.Tensor | None = None
    """Validity mask for fixed taxel slots, shaped ``(N, P)``."""

    num_valid_tactile_points: torch.Tensor | None = None
    """Number of valid taxels per sensor instance, shaped ``(N,)``."""

    penetration_depth: torch.Tensor | None = None
    """Penetration depth in metres, shaped ``(N, P)``."""

    tactile_normal_force: torch.Tensor | None = None
    """Penalty-model normal force, shaped ``(N, P)``."""

    tactile_shear_force: torch.Tensor | None = None
    """Penalty-model shear force in the taxel frame, shaped ``(N, P, 2)``."""
