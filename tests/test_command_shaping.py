import numpy as np
import pytest

from src.command_shaping import CommandShaper, first_order, rate_limit


def test_rate_limit_bounds_step_change():
    assert rate_limit(0.0, 1.0, rate_per_sec=0.5, dt=0.1) == pytest.approx(0.05)
    assert rate_limit(0.0, -1.0, rate_per_sec=0.5, dt=0.1) == pytest.approx(-0.05)


def test_first_order_rejects_invalid_tau():
    with pytest.raises(ValueError):
        first_order(0.0, 1.0, tau=0.0, dt=0.1)


def test_command_shaper_limits_crossfeed():
    shaper = CommandShaper(
        stick_rate=10.0,
        pedal_rate=10.0,
        tau_pilot_1=0.05,
        tau_pilot_2=0.05,
        t_crossfeed=0.6,
        k_crossfeed=10.0,
        crossfeed_limit=0.02,
        tau_cross_smooth=0.01,
    )

    crossfeed_values = []
    for _ in range(100):
        _, _, crossfeed = shaper.step(stick_raw=0.0, pedal_raw=1.0, dt=0.01)
        crossfeed_values.append(crossfeed)

    assert np.max(np.abs(crossfeed_values)) <= 0.020001
