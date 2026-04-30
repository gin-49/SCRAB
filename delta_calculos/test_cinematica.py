from cinematica_inversa import inverse_kinematics
from cinematica_directa import direct_kinematics

# x0, y0, z0 = coordenadas deseadas del end effector
# rf = longitud Bicep
# re = longitud brazo
# f = lado triangulo base fija
# e = lado triangulo base movil

# Ángulos para probar
theta1 = -12.419
theta2 = -15.845
theta3 = -19.928

# Coordenadas para probar
x0 = -9
y0 = 14
z0 = -300


# Parametros del Robot
rf = 250  # bicep
re = 500  # brazo
f = 300  # base
e = 25  # end effector

# cinematica direct
dk = direct_kinematics(f, rf, re, e, theta1, theta2, theta3)
print("---DIRECT KINEMATICS---")
print(f"angles: {theta1}, {theta2}, {theta3}")
print(f"coords: \n x0: {dk[0]}\n y0: {dk[1]}\n z0: {dk[2]}")

# cinematica inversa
ik = inverse_kinematics(f, rf, re, e, x0, y0, z0)
print("---INVERSE KINEMATICS---")
print(f"coordenadas: {x0}, {y0}, {z0}")
print(f"angles: \n motor1: {ik[0]}\n motor2: {ik[1]}\n motor3: {ik[2]}")
