import numpy as np

# Raiz cuadrada de 3
rc3 = 1.73205080757

# Calculo posicion del motor


def calculo_y1(f):
    return (-f) / (2 * rc3)


# Calculo proyeccion de y
def calculo_y2(y0, e):
    return y0 - (e / (2 * rc3))


# Coeficientes Auxiliares (a, b)
# Calculo de b
def calculo_b(y1, y2, z0):
    return (y1 - y2) / z0


# Calculo de a
def calculo_a(x0, y1, y2, z0, rf, re):
    return (x0**2 + y2**2 + z0**2 + rf**2 - re**2 - y1**2) / (2 * z0)


# Calculo de la coordenada Y del codo (yj)
def calculo_y(y1, a, b, rf):
    num = (y1 - a * b) - np.sqrt(-((a + b * y1) ** 2) + (b**2 + 1) * rf**2)
    return num / (b**2 + 1)


# Calculo para un joint
def joint_theta(a, b, y, y1):
    zj1 = a + b * y
    return np.arctan(zj1 / (y1 - y))
