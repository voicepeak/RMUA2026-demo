"""Control: position -> velocity -> PWM.

Reuse main/control/pwm_publisher + motor_allocator + safety_supervisor.
This module is only the outer loop glue; do not re-implement mixing here.
TODO(P2): cascaded P controller with the verified hover chain.
"""

import numpy as np


class Controller:
    def __init__(self, kp: float = 1.0, v_max: float = 6.0) -> None:
        self.kp = kp
        self.v_max = v_max

    def command(self, state, setpoint) -> np.ndarray:
        """TODO(P2): world velocity command, clipped by v_max and safety gates."""
        raise NotImplementedError
