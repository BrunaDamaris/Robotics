import roboticstoolbox as rtb
import math
from math import pi, sin, cos
import numpy as np
import time
import spatialmath as sm
from sqlalchemy import true
from zmqRemoteApi import RemoteAPIClient
import matplotlib.pyplot as plt
from roboticstoolbox import *

l1 =  0.475
l2 =  0.4

#function to build the matrix for each joint
def make_matrix(theta, alpha,a,d):
    matrix = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),a*cos(theta)],
                 [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha),a*sin(theta)],
                 [0, sin(alpha), cos(alpha),d],
                 [0, 0, 0, 1]])
    return matrix

#function to calculate the fkine of each joint of the robot and return
def calculate_fkine(q):
    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2]
    d = q[3]
    #Duas primeiras matrizes correpondentes as duas primeiras juntas
    TR1 = make_matrix(theta1,0,l1,0)
    #print("Primeira Junta:\n",TR1,'\n')
    TR2 = make_matrix(theta2,0,l2,0)
    #print("Segunda Junta:\n",TR2,'\n')
    TR12 = np.matmul(TR1,TR2) #Calcula multiplacao entre as duas matrizes
    #Terceira matriz da terceira junta
    TR3 = make_matrix(theta3,0,0,0)
    #print("Terceira Junta:\n",TR3,'\n')
    TR123 = np.matmul(TR12,TR3) #Calcula multiplicacao entre a matriz resultante da primeira multiplicacao com a da junta 3
    #Quarta junta
    TR4 = make_matrix(0,pi,0,0)
    #print("Quarta Junta:\n",TR4,'\n')
    TR1234 = np.matmul(TR123,TR4) #Calcula multiplicacao entre a matriz resultante da segunda multiplicacao com a da junta 4
    #Ultima junta
    TP1 = make_matrix(0,0,0,d)
    #print("Ultima Junta:\n",TP1,'\n')
    T = np.matmul(TR1234,TP1) #Calcula multiplicacao entre a matriz resultante da terceira multiplicacao com a da junta 5
    return np.matrix.round(T,decimals=5)

def jacobian(q):
    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[3]
    matrix = np.array([[-l2*sin(theta1+theta2)-l1*sin(theta1),-l2*sin(theta1+theta2),0,0],
                       [l2*cos(theta1+theta2)+l1*cos(theta1),l2*cos(theta1+theta2),0,0],
                       [0,0,0,-1],
                       [0,0,0,0],
                       [0,0,0,0],
                       [1,1,1,0]])
    return matrix

client = RemoteAPIClient()
sim = client.getObject('sim')

client.setStepping(True)
sim.startSimulation()

jointR1Handle = sim.getObject('/MTB/axis')
jointR2Handle = sim.getObject('/MTB/link/axis')
jointR3Handle = sim.getObject('/MTB/link/axis/link/axis/axis')
jointP1Handle = sim.getObject('/MTB/link/axis/link/axis')
dummyHandle = sim.getObject('/reference')

X_d1 = sim.getObjectPosition(dummyHandle,-1)
X_d2 = [np.pi,0,sim.getObjectOrientation(dummyHandle,-1)[0]]
X_d = np.hstack([X_d1,X_d2])

q = [0,0,0,0]#theta1, theta2, theta3 e d

q[0] = sim.getJointPosition(jointR1Handle)
q[1] = sim.getJointPosition(jointR2Handle)
q[2] = sim.getJointPosition(jointR3Handle)
q[3] = sim.getJointPosition(jointP1Handle)

X_c = calculate_fkine(q)
X_c = X_c[:,3]
X_c = np.delete(X_c,3)
rpy = [np.pi,0,sim.getObjectOrientation(jointP1Handle,-1)[0]]
X_c = np.hstack([X_c,rpy])

Ts = 0.05
e = 0.07
X_e_list = []
joints_list_1 = []
joints_list_2 = []
joints_list_3 = []
joints_list_4 = []

while True:
    X_e = X_d-X_c
    X_e_list.append(X_d-X_c)

    j = jacobian(q)
    j_inv = np.linalg.pinv(j)

    dq = j_inv@(X_e)
    q += dq*Ts

    joints_list_1.append(q[0])
    joints_list_2.append(q[1])
    joints_list_3.append(q[2])
    joints_list_4.append(q[3])

    sim.setJointPosition(jointR1Handle, q[0])
    time.sleep(0.05)
    sim.setJointPosition(jointR2Handle, q[1])
    time.sleep(0.05)
    sim.setJointPosition(jointR3Handle, q[2])
    time.sleep(0.05)
    sim.setJointPosition(jointP1Handle, q[3])
    time.sleep(0.05)

    X_c = calculate_fkine(q)
    X_c = X_c[:,3]
    X_c = np.delete(X_c,3)
    rpy = [np.pi,0,sim.getObjectOrientation(jointP1Handle,-1)[0]]
    X_c = np.hstack([X_c,rpy])

    X_d1 = sim.getObjectPosition(dummyHandle,-1)
    X_d2 = [np.pi,0,sim.getObjectOrientation(dummyHandle,-1)[0]]
    X_d = np.hstack([X_d1,X_d2])

    print(np.linalg.norm(X_e))
    if(np.linalg.norm(X_e) <= e):
        break


plt.plot(joints_list_1)
plt.title("Joint 1 Positions")
plt.show()
plt.plot(joints_list_2)
plt.title("Joint 2 Positions")
plt.show()
plt.plot(joints_list_3)
plt.title("Joint 3 Positions")
plt.show()
plt.plot(joints_list_4)
plt.title("Joint 4 Positions")
plt.show()

plt.plot(X_e_list, label=['x','y','z','roll','pitch','yaw'])
plt.legend()
plt.title("Error Plot")
plt.show()
sim.stopSimulation()
