import numpy as np

class LinearPlant:
    """Linear plant: x_dot = A x + B u"""

    def __init__(self, A: np.ndarray, B: np.ndarray):
        self.A = A
        self.B = B
        self.x = np.zeros(A.shape[0])

    def reset(self):
        self.x[:] = 0.0

    def step(self, u: np.ndarray, dt: float):
        dx = self.A @ self.x + self.B @ u
        self.x = self.x + dx * dt
        return self.x
