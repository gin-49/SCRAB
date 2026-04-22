import numpy as np


# Primero se calculan el centro de las 3 estras y se intersectan
# para encontrar la posicion del end eftctor, esto se calcula
# solo una vez para ahorrar poder computacional
def direct_kinematics(f, rf, re, e, theta1, theta2, theta3):
    # Convertir a radianes
    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)
    theta3 = np.radians(theta3)

    # Términos repetidos
    t = (f - e) / (2.0 * np.sqrt(3))
    sqrt3 = np.sqrt(3.0)
    sin30 = 0.5
    tan60 = sqrt3

    # Primer esfera
    x1 = 0
    y1 = -(t + rf * np.cos(theta1))
    z1 = -rf * np.sin(theta1)

    # Segunda Esfera
    y2 = (t + rf * np.cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * np.sin(theta2)

    # Tercera Esfera
    y3 = (t + rf * np.cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * np.sin(theta3)

    # Calculo de terminos auxiliares
    dnm = (y2 - y1) * x3 - (y3 - y1) * x2

    w1 = x1**2 + y1**2 + z1**2
    w2 = x2**2 + y2**2 + z2**2
    w3 = x3**2 + y3**2 + z3**2

    # x = (a1*z + b1)/dnm
    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

    # y = (a2*z + b2)/dnm;
    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

    # a*z^2 + b*z + c = 0
    a = a1**2 + a2**2 + dnm**2
    b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)
    d = b**2 - 4.0 * a * c

    if d < 0:
        return "error"

    z0 = -0.5 * (b + np.sqrt(d)) / a
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm

    return x0, y0, z0
