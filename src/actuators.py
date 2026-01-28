import numpy as np

class SecondOrderActuator2Ch:
    """
    Two independent 2nd-order actuator channels.

    States:
      [d1, d1_dot, d2, d2_dot]
    Dynamics per channel:
      d_ddot = -2*zeta*w0*d_dot - w0^2*d + w0^2*u_cmd
    With optional acceleration, rate, and position limits.
    """

    def __init__(
        self,
        w0: np.ndarray,
        zeta: np.ndarray,
        delta_max: np.ndarray,
        rate_max: np.ndarray,
        acc_max: np.ndarray,
    ):
        self.w0 = np.array(w0, dtype=float)
        self.zeta = np.array(zeta, dtype=float)
        self.delta_max = np.array(delta_max, dtype=float)
        self.rate_max = np.array(rate_max, dtype=float)
        self.acc_max = np.array(acc_max, dtype=float)
        self.x = np.zeros(4, dtype=float)

    def reset(self):
        self.x[:] = 0.0

    def step(self, u_cmd: np.ndarray, dt: float) -> np.ndarray:
        d1, d1_dot, d2, d2_dot = self.x
        u1, u2 = float(u_cmd[0]), float(u_cmd[1])

        # Channel 1
        d1_ddot = (-self.w0[0] ** 2) * d1 - (2.0 * self.zeta[0] * self.w0[0]) * d1_dot + (self.w0[0] ** 2) * u1
        # Channel 2
        d2_ddot = (-self.w0[1] ** 2) * d2 - (2.0 * self.zeta[1] * self.w0[1]) * d2_dot + (self.w0[1] ** 2) * u2

        # Acceleration saturation
        d1_ddot = np.clip(d1_ddot, -self.acc_max[0], self.acc_max[0])
        d2_ddot = np.clip(d2_ddot, -self.acc_max[1], self.acc_max[1])

        # Integrate rate
        d1_dot = d1_dot + d1_ddot * dt
        d2_dot = d2_dot + d2_ddot * dt

        # Rate saturation
        d1_dot = np.clip(d1_dot, -self.rate_max[0], self.rate_max[0])
        d2_dot = np.clip(d2_dot, -self.rate_max[1], self.rate_max[1])

        # Integrate position
        d1 = d1 + d1_dot * dt
        d2 = d2 + d2_dot * dt

        # Position saturation
        d1 = np.clip(d1, -self.delta_max[0], self.delta_max[0])
        d2 = np.clip(d2, -self.delta_max[1], self.delta_max[1])

        self.x[:] = [d1, d1_dot, d2, d2_dot]

        # Plant sees actual deflection
        return np.array([d1, d2], dtype=float)
