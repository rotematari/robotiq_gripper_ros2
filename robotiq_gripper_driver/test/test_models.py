import math

import pytest

from robotiq_gripper_driver.models import MODELS, Converter, get_model

HANDE = Converter(get_model("hande"))


def test_width_ends_map_to_open_and_closed_registers():
    assert HANDE.width_to_register(0.050) == 0
    assert HANDE.width_to_register(0.0) == 255
    assert HANDE.register_to_width(0) == pytest.approx(0.050)
    assert HANDE.register_to_width(255) == 0.0


def test_width_uses_calibrated_registers():
    conv = Converter(get_model("hande"), open_register=3, closed_register=245)
    assert conv.width_to_register(0.050) == 3
    assert conv.width_to_register(0.0) == 245
    assert conv.width_to_register(0.025) == 124
    # Readings beyond the calibrated ends clamp to the stroke.
    assert conv.register_to_width(0) == pytest.approx(0.050)
    assert conv.register_to_width(255) == 0.0


@pytest.mark.parametrize("name", sorted(MODELS))
def test_round_trips_stay_within_one_register(name):
    conv = Converter(MODELS[name])
    m = conv.model
    for width in (0.0, m.stroke / 3, m.stroke):
        assert conv.register_to_width(conv.width_to_register(width)) == pytest.approx(
            width, abs=m.stroke / 255
        )
    for force in (m.min_force, (m.min_force + m.max_force) / 2, m.max_force):
        span = m.max_force - m.min_force
        assert conv.register_to_force(conv.force_to_register(force)) == pytest.approx(
            force, abs=span / 255
        )
    assert conv.speed_to_register(m.min_speed) == 0
    assert conv.speed_to_register(m.max_speed) == 255


@pytest.mark.parametrize(
    ("method", "value"),
    [
        ("width_to_register", -0.001),
        ("width_to_register", 0.051),
        ("width_to_register", math.nan),
        ("force_to_register", 19.0),
        ("force_to_register", 131.0),
        ("force_to_register", math.inf),
        ("speed_to_register", 0.0),
        ("speed_to_register", 0.2),
        ("width_to_register", "0.01"),
    ],
)
def test_out_of_range_values_are_rejected(method, value):
    with pytest.raises(ValueError):
        getattr(HANDE, method)(value)


def test_bad_models_and_calibration_are_rejected():
    with pytest.raises(ValueError, match="unknown model"):
        get_model("3f")
    with pytest.raises(ValueError):
        Converter(get_model("hande"), open_register=200, closed_register=100)
    with pytest.raises(ValueError):
        Converter(get_model("hande"), closed_register=256)
