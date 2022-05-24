from zmqRemoteApi import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject('sim')

dummyHandle = sim.getObject('/MTB/Dummy')

client.setStepping(True)
sim.startSimulation()

dummyPose = sim.getObjectPose(dummyHandle,-1)
print(dummyPose)

sim.stopSimulation()