import numpy as np

class LinearPlant:
    """Linear plant: x_dot = A x + B u"""

    def __init__(self, A: np.ndarray, B: np.ndarray):
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float)

        if self.A.ndim != 2 or self.A.shape[0] != self.A.shape[1]:
            raise ValueError("A must be a square state matrix")
        if self.B.ndim != 2 or self.B.shape[0] != self.A.shape[0]:
            raise ValueError("B must have one row per plant state")
        if not np.all(np.isfinite(self.A)) or not np.all(np.isfinite(self.B)):
            raise ValueError("plant matrices must contain finite values")

        self.x = np.zeros(A.shape[0])

    def reset(self):
        self.x[:] = 0.0

    def step(self, u: np.ndarray, dt: float):
        if dt <= 0.0 or not np.isfinite(dt):
            raise ValueError("dt must be a positive finite time step")
        u = np.asarray(u, dtype=float)
        if u.shape != (self.B.shape[1],):
            raise ValueError(f"u must be a {self.B.shape[1]}-element vector")
        if not np.all(np.isfinite(u)):
            raise ValueError("u must contain finite values")

        dx = self.A @ self.x + self.B @ u
        self.x = self.x + dx * dt
        return self.x
