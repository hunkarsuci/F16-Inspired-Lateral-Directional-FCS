import numpy as np

class StateFeedbackController:
    """
    u_cmd = H*[stick, pedal] - K*x + crossfeed injected into channel 1
    """

    def __init__(self, K: np.ndarray, H: np.ndarray, u_cmd_limit: float = 1.0):
        self.K = np.asarray(K, dtype=float)
        self.H = np.asarray(H, dtype=float)
        self.u_cmd_limit = float(u_cmd_limit)

        if self.K.shape[0] != self.H.shape[0]:
            raise ValueError("K and H must produce the same number of control channels")
        if self.H.shape[1] != 2:
            raise ValueError("H must map [stick, pedal] commands")
        if not np.all(np.isfinite(self.K)) or not np.all(np.isfinite(self.H)):
            raise ValueError("controller gains must contain finite values")
        if self.u_cmd_limit <= 0.0 or not np.isfinite(self.u_cmd_limit):
            raise ValueError("u_cmd_limit must be positive and finite")

    def compute(self, stick: float, pedal: float, x: np.ndarray, crossfeed: float) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if x.shape != (self.K.shape[1],):
            raise ValueError(f"x must be a {self.K.shape[1]}-element state vector")
        if not np.all(np.isfinite(x)):
            raise ValueError("x must contain finite values")
        if not np.all(np.isfinite([stick, pedal, crossfeed])):
            raise ValueError("stick, pedal, and crossfeed must be finite")

        u = (self.H @ np.array([stick, pedal])) - (self.K @ x)
        u[0] += crossfeed
        u = np.clip(u, -self.u_cmd_limit, self.u_cmd_limit)
        return u
