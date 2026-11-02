"""Gripper model ranges and conversions between SI units and 0-255 registers.

The force and speed scales are linear approximations: Robotiq publishes the
range ends but no register-to-newton curve (see docs/design.md).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GripperModel:
    """Physical ranges of one gripper model, in m, N and m/s."""

    name: str
    stroke: float
    min_force: float
    max_force: float
    min_speed: float
    max_speed: float


MODELS = {
    # Hand-E manual, Specifications (coupling GRP-CPL-062, basic aluminium fingertips).
    "hande": GripperModel("hande", 0.050, 20.0, 130.0, 0.020, 0.150),
    # Robotiq Adaptive Grippers product sheet.
    "2f85": GripperModel("2f85", 0.085, 20.0, 235.0, 0.020, 0.150),
    "2f140": GripperModel("2f140", 0.140, 10.0, 125.0, 0.030, 0.250),
}


def get_model(name: str) -> GripperModel:
    """Return the model for `name`, or raise ValueError listing the known models."""
    try:
        return MODELS[name]
    except KeyError:
        raise ValueError(f"unknown model {name!r}; expected one of {sorted(MODELS)}") from None


def _check_register(name: str, value: int) -> None:
    if type(value) is not int or not 0 <= value <= 255:
        raise ValueError(f"{name} must be an integer from 0 to 255, got {value!r}")


@dataclass(frozen=True)
class Converter:
    """Convert between SI values and registers for one model and width calibration.

    `open_register` and `closed_register` are the position registers at full
    opening and at zero opening; width is linear between them.
    """

    model: GripperModel
    open_register: int = 0
    closed_register: int = 255

    def __post_init__(self) -> None:
        _check_register("open_register", self.open_register)
        _check_register("closed_register", self.closed_register)
        if self.closed_register <= self.open_register:
            raise ValueError("closed_register must be greater than open_register")

    def width_to_register(self, width: float) -> int:
        """Position register for an opening `width` in m; raise ValueError if out of range."""
        _check_range("width", width, 0.0, self.model.stroke, "m")
        span = self.closed_register - self.open_register
        return round(self.closed_register - width / self.model.stroke * span)

    def register_to_width(self, register: int) -> float:
        """Opening in m for a position register, clamped to [0, stroke]."""
        span = self.closed_register - self.open_register
        fraction = (self.closed_register - register) / span
        return min(max(fraction, 0.0), 1.0) * self.model.stroke

    def force_to_register(self, force: float) -> int:
        """Force register for `force` in N; raise ValueError if out of range."""
        m = self.model
        _check_range("force", force, m.min_force, m.max_force, "N")
        return round((force - m.min_force) / (m.max_force - m.min_force) * 255)

    def register_to_force(self, register: int) -> float:
        m = self.model
        return m.min_force + register / 255 * (m.max_force - m.min_force)

    def speed_to_register(self, speed: float) -> int:
        """Speed register for `speed` in m/s; raise ValueError if out of range."""
        m = self.model
        _check_range("speed", speed, m.min_speed, m.max_speed, "m/s")
        return round((speed - m.min_speed) / (m.max_speed - m.min_speed) * 255)

    def register_to_speed(self, register: int) -> float:
        m = self.model
        return m.min_speed + register / 255 * (m.max_speed - m.min_speed)


def _check_range(name: str, value: float, low: float, high: float, unit: str) -> None:
    # A tolerance of 1e-9 absorbs float round-off at the range ends (for example 0.05 m).
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number, got {value!r}")
    if not low - 1e-9 <= value <= high + 1e-9:
        raise ValueError(f"{name} {value:g} {unit} is outside [{low:g}, {high:g}] {unit}")
