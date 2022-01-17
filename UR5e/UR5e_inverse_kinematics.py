from math import isnan
from numpy import arcsin, array, arctan2, sqrt, pi, arccos, sin, cos, round, linalg

# Many of the calculations are based on the findings in: http://rasmusan.blog.aau.dk/files/ur5_kinematics.pdf

# As the robot has 6 degrees of freedom, there can be some configurations where joint angles can be arbitrary.
# The important thing to look at when validating the inverse kinematics, is if the joint angles calculated result in the correct transformation matrix.

# We have not yet looked at limiting the joint values when calculated the inverse kinematics.

# UR5e MDH parameters
a2 = 0.425
a3 = 0.3922
d1 = 0.1625
d4 = 0.1333
d5 = 0.0997
d6 = 0.0996

# UR5e FK parameters
def UR5eFK(theta1,theta2,theta3,theta4,theta5,theta6):
    T_0_1 = array([[cos(theta1), -sin(theta1), 0, 0],[sin(theta1), cos(theta1), 0, 0],[0, 0, 1, d1],[0,0,0,1]])
    T_1_2 = array([[cos(theta2), -sin(theta2), 0, 0],[0, 0, -1, 0],[sin(theta2), cos(theta2), 0, 0],[0,0,0,1]])
    T_2_3 = array([[cos(theta3), -sin(theta3), 0, a2],[sin(theta3), cos(theta3), 0, 0],[0,0,1,0],[0,0,0,1]])
    T_3_4 = array([[cos(theta4), -sin(theta4), 0, a3],[sin(theta4), cos(theta4), 0, 0],[0, 0, 1, d4],[0,0,0,1]])
    T_4_5 = array([[cos(theta5), -sin(theta5), 0, 0],[0,0,-1,-d5],[sin(theta5), cos(theta5), 0, 0],[0,0,0,1]])
    T_5_6 = array([[cos(theta6), -sin(theta6), 0, 0],[0,0,1,d6],[-sin(theta6),-cos(theta6), 0,0],[0,0,0,1]])
    T_0_6 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6
    return T_0_6


# Calculate forward kinematics from given joint angles. The generated transfromation matrix is used as input to the inverse kinematics algorithm.
q1 = pi/5
q2 = 0
q3 = pi/8
q4 = 0
q5 = pi/2
q6 = pi
q = [q1, q2, q3, q4, q5, q6]
T_0_6 = round(UR5eFK(q1,q2,q3,q4,q5,q6),5)
print(round(T_0_6,3))

# desired position and orientation [m]
x = T_0_6[0][3]
y = T_0_6[1][3]
z = T_0_6[2][3]

# Joint angles
P_0_5 = T_0_6 @ array([0,0,-d6,1]).T

# j1
r1 = sqrt(P_0_5[0]**2 + P_0_5[1]**2)
phi1 = arctan2(P_0_5[1], P_0_5[0])
phi2 = arccos(d4/r1)
j1 = phi1 - phi2 + pi/2


# j5
j5 = arccos((x*sin(j1)-y*cos(j1)-d4)/d6)
if isnan(j5):
    j5 = 0.0

# j6
R_0_6 = T_0_6[0:3,0:3]
X_0_6_rot = R_0_6[:,0]
Y_0_6_rot = R_0_6[:,1]
X_6_0_rot = -X_0_6_rot
Y_6_0_rot = -Y_0_6_rot
num1 = (-Y_6_0_rot[0]*sin(j1) + Y_6_0_rot[1]*cos(j1))
num2 = (X_6_0_rot[0]*sin(j1)-X_6_0_rot[1]*cos(j1))

if round(j5,5) == 0:
    j6 = 0.0
else:
    j6 = arctan2((num1/sin(j5)),(num2/sin(j5))) + pi
if isnan(j6):
    j6 = 0.0


# j3
T_0_1 = array([[cos(j1), -sin(j1), 0, 0],[sin(j1), cos(j1), 0, 0],[0,0,1,d1],[0,0,0,1]])
T_1_6 = linalg.inv(T_0_1) @ T_0_6
T_4_5 = array([[cos(j5), -sin(j5), 0, 0],[0,0,-1,-d5],[sin(j5), cos(j5),0,0],[0,0,0,1]])
T_5_6 = array([[cos(j6), -sin(j6),0,0],[0,0,1,d6],[-sin(j6), -cos(j6), 0, 0],[0,0,0,1]])
T_1_4 = T_1_6 @ linalg.inv(T_4_5 @ T_5_6)
P_1_3 = T_1_4 @ array([0,-d4,0,1]).T - array([0,0,0,1]).T
P_1_4 = T_1_4[0:3,3]
P_1_4_a = -P_1_4[0]
P_1_4_b = -P_1_4[2]
P_1_4_c = sqrt(P_1_4_a**2 + P_1_4_b**2)
phi3 = arccos((P_1_4_c**2 - a2**2 - a3**2)/(-2*a2*a3))
j3 = pi - phi3
if isnan(j3):
    j3 = 0.0
# check that (P_1_4_c == c) in non-right triangle
if sqrt(a2**2+a3**2-2*a2*a3*cos(phi3)) != P_1_4_c:
    print("Woops")

# j2
phi4 = arctan2(-P_1_4_b,-P_1_4_a)
phi5 = arcsin((-a3 * sin(-j3))/P_1_4_c)
j2 = phi4 - phi5

# j4
T_1_2 = array([[cos(j2), -sin(j2), 0, 0],[0, 0, -1, 0],[sin(j2), cos(j2), 0, 0],[0,0,0,1]])
T_2_3 = array([[cos(j3), -sin(j3), 0, a2],[sin(j3), cos(j3), 0, 0],[0,0,1,0],[0,0,0,1]])
T_3_4 = linalg.inv(T_1_2 @ T_2_3) @ T_1_4
X_3_4_rot = T_3_4[0:3,0]
j4 = arctan2(X_3_4_rot[1],X_3_4_rot[0])


#################
#### RESULTS ####
#################

print()
print("Calculated          Expected")
print(f"j1: {round(j1,5)}        q1: {round(q1,5)}")
print(f"j5: {round(j5,5)}        q5: {round(q5,5)}")
print(f"j6: {round(j6,5)}        q6: {round(q6,5)}")
print(f"j3: {round(j3,5)}        q3: {round(q3,5)}")
print(f"j2: {round(j2,5)}        q2: {round(q2,5)}")
print(f"j4: {round(j4,5)}        q4: {round(q4,5)}")


# Calculate forward kinematics from the joints found using the inverse kinematics, and compare with the initial FK transformation matrix
j = [j1, j2, j3, j4, j5, j6]
print()
print("From IK")
print(round(UR5eFK(j1,j2,j3,j4,j5,j6),3))
print()
print("From FK")
print(round(T_0_6,3))