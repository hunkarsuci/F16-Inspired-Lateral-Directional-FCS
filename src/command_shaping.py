import numpy as np

def first_order(x: float, u: float, tau: float, dt: float) -> float:
    """First-order low-pass: x_dot = (u - x)/tau"""
    return x + (u - x) * (dt / max(tau, 1e-9))

def rate_limit(prev: float, cmd: float, rate_per_sec: float, dt: float) -> float:
    """Rate limit scalar signal."""
    max_step = rate_per_sec * dt
    return prev + float(np.clip(cmd - prev, -max_step, max_step))

class CommandShaper:
    """
    Pilot command processing:
      - rate limit (stick, pedal)
      - 2-stage smoothing (LPF)
      - DT1 washout crossfeed (limited + smoothed)
    """

    def __init__(
        self,
        stick_rate: float,
        pedal_rate: float,
        tau_pilot_1: float,
        tau_pilot_2: float,
        t_crossfeed: float,
        k_crossfeed: float,
        crossfeed_limit: float,
        tau_cross_smooth: float,
    ):
        self.stick_rate = stick_rate
        self.pedal_rate = pedal_rate
        self.tau1 = tau_pilot_1
        self.tau2 = tau_pilot_2

        self.Tc = t_crossfeed
        self.Kc = k_crossfeed
        self.crossfeed_limit = crossfeed_limit
        self.tau_cross_smooth = tau_cross_smooth

        # internal states
        self.stick_cmd = 0.0
        self.pedal_cmd = 0.0
        self.stick_f1 = 0.0
        self.stick_f2 = 0.0
        self.pedal_f1 = 0.0
        self.pedal_f2 = 0.0

        self.crossfeed_lag = 0.0
        self.crossfeed_sm = 0.0

    def reset(self):
        self.__init__(
            self.stick_rate, self.pedal_rate,
            self.tau1, self.tau2,
            self.Tc, self.Kc,
            self.crossfeed_limit,
            self.tau_cross_smooth
        )

    def step(self, stick_raw: float, pedal_raw: float, dt: float):
        # Rate limit
        self.stick_cmd = rate_limit(self.stick_cmd, stick_raw, self.stick_rate, dt)
        self.pedal_cmd = rate_limit(self.pedal_cmd, pedal_raw, self.pedal_rate, dt)

        # 2-stage LPF
        self.stick_f1 = first_order(self.stick_f1, self.stick_cmd, self.tau1, dt)
        self.stick_f2 = first_order(self.stick_f2, self.stick_f1,  self.tau2, dt)

        self.pedal_f1 = first_order(self.pedal_f1, self.pedal_cmd, self.tau1, dt)
        self.pedal_f2 = first_order(self.pedal_f2, self.pedal_f1,  self.tau2, dt)

        stick_filt = self.stick_f2
        pedal_filt = self.pedal_f2

        # DT1/washout crossfeed: K*(pedal - LPF(pedal))
        self.crossfeed_lag = first_order(self.crossfeed_lag, pedal_filt, self.Tc, dt)
        crossfeed = self.Kc * (pedal_filt - self.crossfeed_lag)
        crossfeed = float(np.clip(crossfeed, -self.crossfeed_limit, self.crossfeed_limit))
        self.crossfeed_sm = first_order(self.crossfeed_sm, crossfeed, self.tau_cross_smooth, dt)

        return stick_filt, pedal_filt, self.crossfeed_sm
