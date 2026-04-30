from DeltaKinematics import DeltaKinematics

# Parametros del Robot
rf = 250 / 1000  # bicep
re = 500 / 1000  # brazo
f = 300 / 1000  # base
e = 25 / 1000  # end effector

# Ángulos para probar
theta1 = 12.419
theta2 = 15.845
theta3 = 19.928
theta = [theta1, theta2, theta3]

# Coordenadas para probar
x0 = -9
y0 = 14
z0 = -315
pose = [x0 / 1000, y0 / 1000, z0 / 1000]

delta = DeltaKinematics(rf, re, f, e)
fk = delta.fk(theta)
ik = delta.ik(pose)


# cinematica direct
print("---DIRECT KINEMATICS---")
print(f"angles: {theta1}, {theta2}, {theta3}")
print(f"coords: {fk}")

# cinematica inversa
print("---INVERSE KINEMATICS---")
print(f"coordenadas: {x0}, {y0}, {z0}")
print(f"angles: {ik}")
