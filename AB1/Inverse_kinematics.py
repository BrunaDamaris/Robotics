import roboticstoolbox as rtb
import math
from math import pi, sin, cos, sqrt
import numpy as np
from spatialmath import *

def do_robot(l1,l2):
    R1 = rtb.RevoluteDH(a=l1)
    R2 = rtb.RevoluteDH(a=l2)
    R3 = rtb.RevoluteDH(a=0)
    R4 = rtb.RevoluteDH(a=0,alpha=math.pi,qlim = [0, 0])
    P1 = rtb.PrismaticDH(a=0,qlim = [0, 0.1])
    robot = rtb.DHRobot([R1, R2, R3, R4, P1])

    return robot

def func_invkine(x, y, z, phi):
    l1 =  0.475
    l2 =  0.4
    c2 = (x**2+y**2-l1**2-l2**2)/(2*l1*l2)
    
    if(c2 >= -1 and c2 <= 1):
        theta2_1 = np.arctan2(sqrt(1-c2**2),c2)
        theta2_2 = np.arctan2(-sqrt(1-c2**2),c2)

        s2_1 = sqrt(1-c2**2)
        s2_2 = -sqrt(1-c2**2)

        k1 = l2*c2+l1
        k2_1 = l2*s2_1
        k2_2 = l2*s2_2
        
        theta1_1 = np.arctan2(y,x) - np.arctan2(k2_1,k1)
        theta1_2 = np.arctan2(y,x) - np.arctan2(k2_2,k1)
        
        theta3_1 = phi-theta1_1-theta2_1
        theta3_2 = phi-theta1_2-theta2_2
    else:
        print("error")
        return 0
    #Escolhendo entre os angulos usando minimo
    angles_1 = [theta1_1, theta2_1, theta3_1]
    angles_2 = [theta1_2, theta2_2, theta3_2]
    thetas = [angles_1,angles_2]
    solution = min(thetas)
    theta1 = solution[0]
    theta2 = solution[1]
    theta3 = solution[2]
    d4 = z
    return theta1, theta2, theta3, d4

l1 =  0.475
l2 =  0.4
robot = do_robot(l1,l2)

print("\n\n- (0.2, 0.1, -0.015, pi/4)")
t1,t2,t3,d = func_invkine(0.2, 0.1, -0.015, pi/4)
print("Theta 1 em rad: ",t1)
print("Theta 2 em rad: ",t2)
print("Theta 3 em rad: ",t3)
print("Theta 1 em graus: ",t1*180/pi)
print("Theta 2 em graus: ",t2*180/pi)
print("Theta 3 em graus: ",t3*180/pi)
print("d: ",d)

print("\n\n- (0.5, 0.1, -0.015, pi/4)")
t1,t2,t3,d = func_invkine(0.5, 0.1, -0.015, pi/4)
print("Theta 1 em rad: ",t1)
print("Theta 2 em rad: ",t2)
print("Theta 3 em rad: ",t3)
print("Theta 1 em graus: ",t1*180/pi)
print("Theta 2 em graus: ",t2*180/pi)
print("Theta 3 em graus: ",t3*180/pi)
print("d: ",d)

print("\n\n- (0.15, 0.15, 0, pi)")
t1,t2,t3,d = func_invkine(0.15, 0.15, 0, pi/4)
print("Theta 1 em rad: ",t1)
print("Theta 2 em rad: ",t2)
print("Theta 3 em rad: ",t3)
print("Theta 1 em graus: ",t1*180/pi)
print("Theta 2 em graus: ",t2*180/pi)
print("Theta 3 em graus: ",t3*180/pi)
print("d: ",d)