import roboticstoolbox as rtb
import math
from math import pi, sin, cos
import numpy as np
import time
from zmqRemoteApi import RemoteAPIClient

#function to do robot
def do_robot(l1,l2):
    R1 = rtb.RevoluteDH(a=l1)
    R2 = rtb.RevoluteDH(a=l2)
    R3 = rtb.RevoluteDH(a=0)
    R4 = rtb.RevoluteDH(a=0,alpha=math.pi,qlim = [0, 0])
    P1 = rtb.PrismaticDH(a=0,qlim = [0, 0.1])
    robot = rtb.DHRobot([R1, R2, R3, R4, P1])

    #robot.teach()
    return robot

#function to build the matrix for each joint
def make_matrix(theta, alpha,a,d):
    matrix = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),a*cos(theta)],
                 [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha),a*sin(theta)],
                 [0, sin(alpha), cos(alpha),d],
                 [0, 0, 0, 1]])
    return matrix

#function to calculate the fkine of each joint of the robot and return
def calculate_fkine(theta1,theta2,theta3,d,l1,l2):
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

#Start  
l1 =  0.475
l2 =  0.4
robot = do_robot(l1,l2)
print("A tabela de parâmetros DH")
print(robot,'\n\n')


#print("Print do objeto SerialLink gerado na robotics toolbox (link em anexo)")
#robot.teach()

#Letra a
print("\n(a) theta_1 = 0; theta_2 = 0; theta_3 = 0; d_4 = 0")
theta1 = 0
theta2 = 0
theta3 = 0
d_4 = 0
print('\n','Resultado FKine:\n',calculate_fkine(theta1,theta2,theta3,d_4,l1,l2))

#Letra b
print("\n(b) theta_1 = pi/2; theta_2 = -pi/2; theta_3 = 0; d_4 = 0")
theta1 = pi/2
theta2 = -pi/2
theta3 = 0
d_4 = 0
print('\n','Resultado FKine:\n',calculate_fkine(theta1,theta2,theta3,d_4,l1,l2))

#Letra c
print("\n(c) theta_1 = pi/2; theta_2 = -pi/2; theta_3 = 0; d_4 = 0.05")
theta1 = pi/2
theta2 = -pi/2
theta3 = 0
d_4 = 0.05
print('\n','Resultado FKine:\n',calculate_fkine(theta1,theta2,theta3,d_4,l1,l2))

print('\n\nFKine do RoboticToolBox:\n')
#Letra a
print("\n(a) theta_1 = 0; theta_2 = 0; theta_3 = 0; d_4 = 0")
theta1 = 0
theta2 = 0
theta3 = 0
d_4 = 0

FkineA = robot.fkine([theta1,theta2,theta3,0,d_4])
print('\n',FkineA)

#Letra b
print("\n(b) theta_1 = pi/2; theta_2 = -pi/2; theta_3 = 0; d_4 = 0")
theta1 = pi/2
theta2 = -pi/2
theta3 = 0
d_4 = 0

FkineB = robot.fkine([theta1,theta2,theta3,0,d_4])
print('\n',FkineB)

#Letra c
print("\n(c) theta_1 = pi/2; theta_2 = -pi/2; theta_3 = 0; d_4 = 0.05")
theta1 = pi/2
theta2 = -pi/2
theta3 = 0
d_4 = 0.05

FkineC = robot.fkine([theta1,theta2,theta3,0,d_4])
print('\n',FkineC)


print("\n\nEnviar comandos para o CoppeliaSim")

client = RemoteAPIClient()
sim = client.getObject('sim')

jointR1Handle = sim.getObject('/MTB/axis')
jointR2Handle = sim.getObject('/MTB/link/axis')
jointR3Handle = sim.getObject('/MTB/link/axis/link/axis/axis')
jointP1Handle = sim.getObject('/MTB/link/axis/link/axis')

client.setStepping(True)

sim.startSimulation()

print("Letra a")
theta1 = 0
theta2 = 0
theta3 = 0
d_4 = 0
sim.setJointPosition(jointR1Handle,theta1)
sim.setJointPosition(jointR2Handle,theta2)
sim.setJointPosition(jointR3Handle,theta3)
sim.setJointPosition(jointP1Handle, d_4)

time.sleep(5)

print("Letra b")
theta1 = pi/2
theta2 = -pi/2
theta3 = 0
d_4 = 0
sim.setJointPosition(jointR1Handle,theta1)
sim.setJointPosition(jointR2Handle,theta2)
sim.setJointPosition(jointR3Handle,theta3)
sim.setJointPosition(jointP1Handle, d_4)

time.sleep(5)

print("Letra c")
theta1 = pi/2
theta2 = -pi/2
theta3 = 0
d_4 = 0.05
sim.setJointPosition(jointR1Handle,theta1)
sim.setJointPosition(jointR2Handle,theta2)
sim.setJointPosition(jointR3Handle,theta3)
sim.setJointPosition(jointP1Handle, d_4)

time.sleep(5)

sim.stopSimulation()