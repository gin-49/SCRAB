import numpy as np


# Genera theta para un brazo
def theta_function(f, rf, re, e, x0, y0, z0):
    # Raiz cuadrada de 3
    rc3 = np.sqrt(3)

    # Posicion del motor
    y1 = (-f) / (2 * rc3)

    # Proyeccion de y
    y2 = y0 - (e / (2 * rc3))

    # Coeficientes auxiliares a y b
    a = (x0**2 + y2**2 + z0**2 + rf**2 - re**2 - y1**2) / (2 * z0)
    b = (y1 - y2) / z0

    # Coordenada Y del codo (yj)
    num_yj1 = (y1 - a * b) - np.sqrt((-((a + b * y1) ** 2)) + (b**2 + 1) * rf**2)
    yj1 = num_yj1 / (b**2 + 1)

    # Distancia eje z entre joint y base
    zj1 = a + b * yj1

    # Angulo entre base y joint
    theta = np.arctan(zj1 / (y1 - yj1))

    return theta


# Rotacion de theta en 120 y 240 grados
def inverse_kinematics(f, rf, re, e, x0, y0, z0):
    sen120 = np.sqrt(3) / 2.0
    # para brazo 1 (0)
    # para brazo 2 (120)
    x120 = x0 * -0.5 + y0 * sen120
    y120 = y0 * -0.5 - x0 * sen120
    z120 = z0

    # para brazo 3 (240)
    x240 = x0 * -0.5 - y0 * sen120
    y240 = y0 * -0.5 + x0 * sen120
    z240 = z0

    theta1 = np.degrees(theta_function(f, rf, re, e, x0, y0, z0))
    theta2 = np.degrees(theta_function(f, rf, re, e, x120, y120, z120))
    theta3 = np.degrees(theta_function(f, rf, re, e, x240, y240, z240))

    return theta1, theta2, theta3
