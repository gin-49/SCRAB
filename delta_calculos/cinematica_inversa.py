import numpy as np
from delta_calculations.constants import LADO_BASE


# Codigo para termino comun de la ecuacion
def common_term(a, b, y1, rf):
    return np.sqrt((a + b * y1) ** 2 + (b**2 + 1) * rf**2)


# Calculo para un joint
def joint_theta(a, b, y1, rf):
    sqrt_term = common_term(a, b, y1, rf)

    second_term = ((y1 - a * b) - sqrt_term) / (b**2 + 1)

    third_term = a + b * ((y1 - a * b) - sqrt_term) / (b**2 + 1)

    yF1 = -((LADO_BASE) / (2 * np.sqrt(3)))

    j1 = np.array([0.0, second_term, third_term])

    theta = np.arctan((a + b * (y * j1)) / yF1 - y * j1)

    return theta
