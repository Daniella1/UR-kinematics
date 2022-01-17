import roboticstoolbox as rtb
from numpy import pi, cos, sin, array, ndarray

# UR5e Modified Denavit-Hartenberg parameters
a2 = 0.425
a3 = 0.3922
d1 = 0.1625
d4 = 0.1333
d5 = 0.0997
d6 = 0.0996

# UR5e kinematics using Robotics Toolbox
class UR5e(rtb.DHRobot):
    def __init__(self):
        L1 = rtb.RevoluteMDH(d=d1, a=0, alpha=0)
        L2 = rtb.RevoluteMDH(d=0, a=0, alpha=pi/2)
        L3 = rtb.RevoluteMDH(d=0, a=a2, alpha=0)
        L4 = rtb.RevoluteMDH(d=d4, a=a3, alpha=0)
        L5 = rtb.RevoluteMDH(d=d5, a=0, alpha=pi/2)
        L6 = rtb.RevoluteMDH(d=d6, a=0, alpha=-pi/2)
        super().__init__([L1, L2, L3, L4, L5, L6],name="UR5e")


# UR5e Forward kinematics transformation
def UR5eFK(j1, j2, j3, j4 , j5, j6):
    T_0_1 = array([[cos(j1), -sin(j1), 0, 0],[sin(j1), cos(j1), 0, 0],[0, 0, 1, d1],[0,0,0,1]])
    T_1_2 = array([[cos(j2), -sin(j2), 0, 0],[0, 0, -1, 0],[sin(j2), cos(j2), 0, 0],[0,0,0,1]])
    T_2_3 = array([[cos(j3), -sin(j3), 0, a2],[sin(j3), cos(j3), 0, 0],[0,0,1,0],[0,0,0,1]])
    T_3_4 = array([[cos(j4), -sin(j4), 0, a3],[sin(j4), cos(j4), 0, 0],[0, 0, 1, d4],[0,0,0,1]])
    T_4_5 = array([[cos(j5), -sin(j5), 0, 0],[0,0,-1,-d5],[sin(j5), cos(j5), 0, 0],[0,0,0,1]])
    T_5_6 = array([[cos(j6), -sin(j6), 0, 0],[0,0,1,d6],[-sin(j6),-cos(j6), 0,0],[0,0,0,1]])
    T_0_6 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6
    print(ndarray.round(T_0_6,4))


sb = UR5e()
q = [pi/2,0,0,pi/3,0,0]
print("Forward Kinematics from MDH parameters and Robotics Toolbox:")
print(sb.fkine(q))
print("Forward Kinematics from manually defined transformation matrices:")
UR5eFK(q[0], q[1], q[2], q[3], q[4], q[5])