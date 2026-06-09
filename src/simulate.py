import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .params import (
    A_PLANT, B_PLANT,
    K_GAINS, H_GAINS,
    STICK_RATE, PEDAL_RATE,
    TAU_PILOT_1, TAU_PILOT_2,
    T_CROSSFEED, K_CROSSFEED, CROSSFEED_LIMIT, TAU_CROSS_SMOOTH,
    U_CMD_LIMIT,
    W0, ZETA, DELTA_MAX, RATE_MAX, ACC_MAX,
)

from .plant import LinearPlant
from .actuators import SecondOrderActuator2Ch
from .command_shaping import CommandShaper
from .controller import StateFeedbackController


def demo_pilot_inputs(time_s: float) -> tuple[float, float]:
    """Thirty-second demo profile with roll and pedal activity throughout."""
    stick_raw = 0.0
    pedal_raw = 0.0

    if time_s < 3.0:
        stick_raw = 0.45
    elif time_s < 6.0:
        stick_raw = -0.25
    elif time_s < 9.0:
        pedal_raw = 0.30
    elif time_s < 12.0:
        stick_raw = -0.40
    elif time_s < 15.0:
        pedal_raw = -0.25
    elif time_s < 18.0:
        stick_raw = 0.30
        pedal_raw = 0.18
    elif time_s < 21.0:
        stick_raw = -0.30
        pedal_raw = -0.18
    elif time_s < 24.0:
        stick_raw = 0.20
    elif time_s < 27.0:
        pedal_raw = 0.22
    else:
        stick_raw = -0.15
        pedal_raw = -0.12

    return stick_raw, pedal_raw


def run_simulation(duration: float = 30.0, steps: int = 5000, out_dir: str = "outputs"):
    if duration <= 0.0 or not np.isfinite(duration):
        raise ValueError("duration must be positive and finite")
    if steps < 2:
        raise ValueError("steps must be at least 2")

    t = np.linspace(0.0, duration, steps)
    dt = float(t[1] - t[0])

    os.makedirs(out_dir, exist_ok=True)

    plant = LinearPlant(A_PLANT, B_PLANT)
    actuators = SecondOrderActuator2Ch(W0, ZETA, DELTA_MAX, RATE_MAX, ACC_MAX)
    shaper = CommandShaper(
        stick_rate=STICK_RATE,
        pedal_rate=PEDAL_RATE,
        tau_pilot_1=TAU_PILOT_1,
        tau_pilot_2=TAU_PILOT_2,
        t_crossfeed=T_CROSSFEED,
        k_crossfeed=K_CROSSFEED,
        crossfeed_limit=CROSSFEED_LIMIT,
        tau_cross_smooth=TAU_CROSS_SMOOTH,
    )
    ctrl = StateFeedbackController(K_GAINS, H_GAINS, u_cmd_limit=U_CMD_LIMIT)

    log_t, log_beta, log_p, log_r, log_phi = [], [], [], [], []
    log_u_cmd, log_u_actual = [], []

    print("Running modular simulation...")

    for ti in t:
        stick_raw, pedal_raw = demo_pilot_inputs(float(ti))

        stick_filt, pedal_filt, crossfeed = shaper.step(stick_raw, pedal_raw, dt)

        u_cmd = ctrl.compute(stick_filt, pedal_filt, plant.x, crossfeed)

        u_actual = actuators.step(u_cmd, dt)

        x = plant.step(u_actual, dt)

        log_t.append(float(ti))
        log_beta.append(float(x[0]))
        log_p.append(float(x[1]))
        log_r.append(float(x[2]))
        log_phi.append(float(x[3]))
        log_u_cmd.append(u_cmd.copy())
        log_u_actual.append(u_actual.copy())

    history = {
        "time_s": np.array(log_t),
        "beta_rad": np.array(log_beta),
        "p_rad_s": np.array(log_p),
        "r_rad_s": np.array(log_r),
        "phi_rad": np.array(log_phi),
        "u_cmd_rad": np.vstack(log_u_cmd),
        "u_actual_rad": np.vstack(log_u_actual),
    }

    log_phi_deg = np.degrees(history["phi_rad"])
    log_p_deg = np.degrees(history["p_rad_s"])

    if not np.all(np.isfinite(log_phi_deg)) or not np.all(np.isfinite(log_p_deg)):
        raise FloatingPointError("simulation produced non-finite states")

    if np.max(np.abs(history["u_actual_rad"])) > float(np.max(DELTA_MAX)) + 1e-9:
        raise FloatingPointError("actuator output exceeded configured position limits")

    # Plot
    plt.figure(figsize=(10, 8))

    plt.subplot(2, 1, 1)
    plt.plot(log_t, log_p_deg, linewidth=2.5)
    plt.title("Roll Rate (p) - 2nd-Order Actuator (Per Channel)")
    plt.ylabel("deg/s")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(log_t, log_phi_deg, linewidth=2.5)
    plt.title("Bank Angle (phi)")
    plt.ylabel("Degrees")
    plt.xlabel("Time (s)")
    plt.grid(True)

    plt.tight_layout()
    out_path = os.path.join(out_dir, "f16_2nd_order_actuator.png")
    plt.savefig(out_path)
    plt.close()
    print(f"SUCCESS: Plot saved to: {os.path.abspath(out_path)}")
    return history
