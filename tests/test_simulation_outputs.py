import os

import numpy as np
import pytest

from src.params import DELTA_MAX
from src.simulate import demo_pilot_inputs, run_simulation
from src.visualization import create_aircraft_animation


def test_simulation_returns_finite_limited_history(tmp_path):
    history = run_simulation(duration=1.0, steps=200, out_dir=str(tmp_path))

    expected_keys = {
        "time_s",
        "beta_rad",
        "p_rad_s",
        "r_rad_s",
        "phi_rad",
        "u_cmd_rad",
        "u_actual_rad",
    }
    assert expected_keys.issubset(history)
    assert history["time_s"].shape == (200,)
    assert history["u_cmd_rad"].shape == (200, 2)
    assert history["u_actual_rad"].shape == (200, 2)

    for value in history.values():
        assert np.all(np.isfinite(value))

    assert np.max(np.abs(history["u_actual_rad"])) <= np.max(DELTA_MAX) + 1e-9
    assert os.path.exists(tmp_path / "f16_2nd_order_actuator.png")


def test_simulation_rejects_too_few_steps(tmp_path):
    with pytest.raises(ValueError):
        run_simulation(duration=1.0, steps=1, out_dir=str(tmp_path))


def test_demo_pilot_inputs_continue_through_30_seconds():
    sample_times = [1.0, 5.0, 8.0, 11.0, 14.0, 17.0, 20.0, 23.0, 26.0, 29.0]
    commands = [demo_pilot_inputs(time_s) for time_s in sample_times]

    assert all(abs(stick) > 0.0 or abs(pedal) > 0.0 for stick, pedal in commands)
    assert len(set(commands)) > 4


def test_3d_animation_file_is_created(tmp_path):
    history = run_simulation(duration=0.4, steps=80, out_dir=str(tmp_path))
    out_path = create_aircraft_animation(
        history,
        out_path=str(tmp_path / "attitude.gif"),
        fps=12,
        max_frames=24,
    )

    assert os.path.exists(out_path)
    assert os.path.getsize(out_path) > 0


def test_3d_animation_supports_alternate_aircraft_model(tmp_path):
    history = run_simulation(duration=0.3, steps=60, out_dir=str(tmp_path))
    out_path = create_aircraft_animation(
        history,
        out_path=str(tmp_path / "concept.gif"),
        fps=12,
        max_frames=18,
        aircraft_model="concept",
    )

    assert os.path.exists(out_path)
    assert os.path.getsize(out_path) > 0
