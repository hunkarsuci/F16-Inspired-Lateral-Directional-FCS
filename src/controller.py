import numpy as np

class StateFeedbackController:
    """
    u_cmd = H*[stick, pedal] - K*x + crossfeed injected into channel 1
    """

    def __init__(self, K: np.ndarray, H: np.ndarray, u_cmd_limit: float = 1.0):
        self.K = K
        self.H = H
        self.u_cmd_limit = float(u_cmd_limit)

    def compute(self, stick: float, pedal: float, x: np.ndarray, crossfeed: float) -> np.ndarray:
        u = (self.H @ np.array([stick, pedal])) - (self.K @ x)
        u[0] += crossfeed
        u = np.clip(u, -self.u_cmd_limit, self.u_cmd_limit)
        return u
