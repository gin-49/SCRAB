import numpy as np


# Codigo para termino comun de la ecuacion
def common_term(a, b, y1, rf):
    return np.sqrt((a + b * y1) ** 2 + (b**2 + 1) * rf**2)


# Calculo para un joint
def joint_theta(a, b, y1, rf):
    sqrt_term = common_term(a, b, y1, rf)

    second_term = ((y1 - a * b) - sqrt_term) / (b**2 + 1)

    third_term = a + b * second_term

    J1 = [0, second_term, third_term]

    theta = np.arctan

    return theta
