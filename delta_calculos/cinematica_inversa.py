import numpy as np


# Genera theta para un brazo
def theta_function(f, rf, re, e, x0, y0, z0):
    rc3 = np.sqrt(3)
    y1 = -(f - e) / (2 * rc3)  # FIX 1: include e offset
    y2 = y0 - (e / (2 * rc3))
    a = (x0**2 + y2**2 + z0**2 + rf**2 - re**2 - y1**2) / (2 * z0)
    b = (y1 - y2) / z0
    disc = -((a + b * y1) ** 2) + (b**2 + 1) * rf**2
    if disc < 0:
        return None
    yj1 = ((y1 - a * b) - np.sqrt(disc)) / (b**2 + 1)
    zj1 = a + b * yj1
    return np.arctan2(-zj1, y1 - yj1)  # FIX 2: atan2 for correct quadrant


def inverse_kinematics(f, rf, re, e, x0, y0, z0):
    s = np.sqrt(3) / 2.0
    x120 = x0 * -0.5 + y0 * s
    y120 = y0 * -0.5 - x0 * s
    z120 = z0  # FIX 3
    x240 = x0 * -0.5 - y0 * s
    y240 = y0 * -0.5 + x0 * s
    z240 = z0  # FIX 3
    theta1 = -np.degrees(theta_function(f, rf, re, e, x0, y0, z0))
    theta2 = -np.degrees(theta_function(f, rf, re, e, x120, y120, z120))
    theta3 = -np.degrees(theta_function(f, rf, re, e, x240, y240, z240))
    return theta1, theta2, theta3
