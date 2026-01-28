import numpy as np

# ==========================================
# PLANT (F-16-style lateral-directional)
# ==========================================
A_PLANT = np.array([
    [-0.322, 0.064, -0.991, 0.048],
    [-30.64, -3.678, 0.665, 0.000],
    [5.861, -0.032, -0.476, 0.000],
    [0.000, 1.000, 0.076, 0.000]
])

B_PLANT = np.array([
    [0.000, 0.004],
    [-73.14, 3.186],
    [1.954, -4.895],
    [0.000, 0.000]
])

# ==========================================
# CONTROL GAINS
# ==========================================
K_GAINS = np.array([[0.00, -0.15, 0.00, -0.80],
                    [2.00,  0.00, -0.50, 0.00]])

H_GAINS = np.array([[-0.5, 0.0],
                    [0.0, -1.0]])

# ==========================================
# COMMAND SHAPING
# ==========================================
STICK_RATE = 0.60
PEDAL_RATE = 0.50

TAU_PILOT_1 = 0.18
TAU_PILOT_2 = 0.18

T_CROSSFEED = 0.60
K_CROSSFEED = 0.80
CROSSFEED_LIMIT = 0.25
TAU_CROSS_SMOOTH = 0.08

# FCC command saturation (optional)
U_CMD_LIMIT = 1.0

# ==========================================
# ACTUATOR (2nd-order per channel)
# Model:
#   x1 = delta, x2 = delta_dot, u = delta_c
#   delta_ddot = -2*zeta*w0*delta_dot - w0^2*delta + w0^2*u
# ==========================================
W0 = np.array([60.0, 60.0])     # rad/s
ZETA = np.array([0.70, 0.70])   # damping

DELTA_MAX = np.array([0.40, 0.40])  # rad
RATE_MAX  = np.array([1.5,  1.5 ])  # rad/s
ACC_MAX   = np.array([50.0, 50.0])  # rad/s^2
