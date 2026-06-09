import numpy as np
import pytest

from src.controller import StateFeedbackController


def test_controller_saturates_command():
    controller = StateFeedbackController(
        K=np.zeros((2, 4)),
        H=np.array([[10.0, 0.0], [0.0, -10.0]]),
        u_cmd_limit=0.5,
    )

    command = controller.compute(stick=1.0, pedal=1.0, x=np.zeros(4), crossfeed=1.0)

    assert np.all(command <= 0.5)
    assert np.all(command >= -0.5)


def test_controller_rejects_bad_state_shape():
    controller = StateFeedbackController(K=np.zeros((2, 4)), H=np.zeros((2, 2)))

    with pytest.raises(ValueError):
        controller.compute(stick=0.0, pedal=0.0, x=np.zeros(3), crossfeed=0.0)
