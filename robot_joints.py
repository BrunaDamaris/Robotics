from zmqRemoteApi import RemoteAPIClient
import math

client = RemoteAPIClient()
sim = client.getObject('sim')

jointR1Handle = sim.getObject('/MTB/axis')
jointR2Handle = sim.getObject('/MTB/link/axis')
jointR3Handle = sim.getObject('/MTB/link/axis/link/axis/axis')
jointP1Handle = sim.getObject('/MTB/link/axis/link/axis')

client.setStepping(True)

sim.startSimulation()
sim.setJointPosition(jointR1Handle,90 * math.pi / 180)
sim.setJointPosition(jointR2Handle,45 * math.pi / 180)
sim.setJointPosition(jointR3Handle,30 * math.pi / 180)
sim.setJointPosition(jointP1Handle, 0.1)

#sim.stopSimulation()
