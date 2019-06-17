from __future__ import division
import sys

sys.path.append('../lib')
# print(sys.path)


import numpy as np
import math
import vrep
import time
# set joint info
RAD2DEG = 180 / math.pi
tstep = 0.005
jointNum=6
baseName = 'Jaco'
jointName = 'Jaco_joint'

# initiate
print('Program started')

# close all vrep-app
vrep.simxFinish(-1)
# connect to vrep-server
while True:

    clientID = vrep.simxStart('127.0.0.1',19997, True, True, 5000, 5)
    if clientID > -1:
        break
    else:
        time.sleep(0.2)
        print("Failed connecting to remoteAPI server!")
print("Connection success!")

# configuration
# set simulatio time step
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tstep, vrep.simx_opmode_oneshot)
# open synchronous model
vrep.simxSynchronous(clientID, True)
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# read handles of base and joint

jointHandle = np.zeros((jointNum,), dtype=np.int)
for i in range(jointNum):
    # print("i:",i)
    _, returnHandle = vrep.simxGetObjectHandle(clientID, jointName + str(i+1), vrep.simx_opmode_blocking)
    jointHandle[i] = returnHandle
_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)

print('Handles available!')

jointConfig = np.zeros((jointNum,))
for i in range(jointNum):
    _, jpos = vrep.simxGetJointPosition(clientID, jointHandle[i], vrep.simx_opmode_streaming)
    jointConfig[i] = jpos

# print('Handle avaliable')
# simulation
lastCmdTime=vrep.simxGetLastCmdTime(clientID)
vrep.simxSynchronousTrigger(clientID)
print('control')
target_pos = np.array([80,160,80,30,20,80])
while vrep.simxGetConnectionId(clientID) != -1:
    currCmdTime=vrep.simxGetLastCmdTime(clientID)
    dt = currCmdTime - lastCmdTime

    for i in range(jointNum):
        _, jpos = vrep.simxGetJointPosition(clientID, jointHandle[i], vrep.simx_opmode_buffer)
        print('position', round(jpos * RAD2DEG, 2))
        jointConfig[i] = jpos

    vrep.simxPauseCommunication(clientID, True)

    for i in range(jointNum):
        target_pos_s = target_pos[i]/RAD2DEG;
        print("target",target_pos[i])
        vrep.simxSetJointTargetPosition(clientID, jointHandle[i], target_pos_s, vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

    lastCmdTime=currCmdTime
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)
