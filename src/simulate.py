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


def run_simulation(duration: float = 10.0, steps: int = 5000, out_dir: str = "outputs"):
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

    log_t, log_phi, log_p = [], [], []

    print("Running modular simulation...")

    for ti in t:
        # Pilot steps (same as your original)
        stick_raw = 0.5 if ti < 2.0 else 0.0
        pedal_raw = 0.3 if 5.0 < ti < 7.0 else 0.0

        stick_filt, pedal_filt, crossfeed = shaper.step(stick_raw, pedal_raw, dt)

        u_cmd = ctrl.compute(stick_filt, pedal_filt, plant.x, crossfeed)

        u_actual = actuators.step(u_cmd, dt)

        x = plant.step(u_actual, dt)

        log_t.append(float(ti))
        log_phi.append(np.degrees(x[3]))
        log_p.append(np.degrees(x[1]))

    # Plot
    plt.figure(figsize=(10, 8))

    plt.subplot(2, 1, 1)
    plt.plot(log_t, log_p, linewidth=2.5)
    plt.title("Roll Rate (p) - 2nd-Order Actuator (Per Channel)")
    plt.ylabel("deg/s")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(log_t, log_phi, linewidth=2.5)
    plt.title("Bank Angle (phi)")
    plt.ylabel("Degrees")
    plt.xlabel("Time (s)")
    plt.grid(True)

    plt.tight_layout()
    out_path = os.path.join(out_dir, "f16_2nd_order_actuator.png")
    plt.savefig(out_path)
    print(f"SUCCESS: Plot saved to: {os.path.abspath(out_path)}")
