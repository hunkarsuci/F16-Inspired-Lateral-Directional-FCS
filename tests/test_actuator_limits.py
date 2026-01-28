import numpy as np
from src.actuators import SecondOrderActuator2Ch

def test_actuator_respects_limits():
    act = SecondOrderActuator2Ch(
        w0=np.array([60.0, 60.0]),
        zeta=np.array([0.7, 0.7]),
        delta_max=np.array([0.4, 0.4]),
        rate_max=np.array([1.5, 1.5]),
        acc_max=np.array([50.0, 50.0]),
    )

    dt = 0.001
    # Command a huge step for 2 seconds
    for _ in range(int(2.0 / dt)):
        u = act.step(np.array([10.0, -10.0]), dt)
        # plant input must respect position limits
        assert abs(u[0]) <= 0.4000001
        assert abs(u[1]) <= 0.4000001

    # Also check internal rates are limited
    d1, d1_dot, d2, d2_dot = act.x
    assert abs(d1_dot) <= 1.5000001
    assert abs(d2_dot) <= 1.5000001
