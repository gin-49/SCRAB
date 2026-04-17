import numpy as np
from cinematica_inversa import inverse_kinematics

# x0, y0, z0 = coordenadas deseadas del end effector
# rf = longitud Bicep
# re = longitud brazo
# f = lado triangulo base fija
# e = lado triangulo base movil


# Coordenadas para probar
x0 = 20
y0 = -40
z0 = -450


# Parametros del Robot
rf = 250
re = 500
f = 300
e = 25

# cinematica inversa
ik = inverse_kinematics(f, rf, re, e, x0, y0, z0)
print(f"angles: \n motor1: {ik[0]}\n motor2: {ik[1]}\n motor3: {ik[2]}")
