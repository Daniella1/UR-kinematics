from sympy import Symbol, Matrix, cos, sin, pi

# MDH parameters
a2 = 0.425
a3 = 0.3922
d1 = 0.1625
d4 = 0.1333
d5 = 0.0997
d6 = 0.0996

theta1 = Symbol('theta1')
theta2 = Symbol('theta2')
theta3 = Symbol('theta3')
theta4 = Symbol('theta4')
theta5 = Symbol('theta5')
theta6 = Symbol('theta6')

T_0_1 = Matrix([[cos(theta1), -sin(theta1), 0, 0],[sin(theta1), cos(theta1), 0, 0],[0, 0, 1, d1],[0,0,0,1]])
T_1_2 = Matrix([[cos(theta2), -sin(theta2), 0, 0],[0, 0, -1, 0],[sin(theta2), cos(theta2), 0, 0],[0,0,0,1]])
T_2_3 = Matrix([[cos(theta3), -sin(theta3), 0, a2],[sin(theta3), cos(theta3), 0, 0],[0,0,1,0],[0,0,0,1]])
T_3_4 = Matrix([[cos(theta4), -sin(theta4), 0, a3],[sin(theta4), cos(theta4), 0, 0],[0, 0, 1, d4],[0,0,0,1]])
T_4_5 = Matrix([[cos(theta5), -sin(theta5), 0, 0],[0,0,-1,-d5],[sin(theta5), cos(theta5), 0, 0],[0,0,0,1]])
T_5_6 = Matrix([[cos(theta6), -sin(theta6), 0, 0],[0,0,1,d6],[-sin(theta6),-cos(theta6), 0,0],[0,0,0,1]])
T_0_6 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6

print(T_0_6)