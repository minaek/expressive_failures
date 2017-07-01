import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()

import openravepy
from openravepy import *
import trajoptpy
import json
import numpy as np
from numpy import *
import trajoptpy.kin_utils as ku
import time 
import random


env = openravepy.Environment()
env.SetViewer('qtcoin')

env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
#env.Load("../data/table.xml")

robot = env.GetRobots()[0]
manip = robot.GetManipulator("rightarm")

robot.SetActiveManipulator(manip)
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
if not ikmodel.load():
    ikmodel.autogenerate()

with robot: # lock environment and save robot state
    robot.SetDOFValues([0,-2,0,0,0,0,0],manip.GetArmIndicies()) # set the first 4 dof values
    Tee = manip.GetEndEffectorTransform() # get end effector
    ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions

h = env.plot3(Tee[0:3,3],10) # plot one point
with robot: # save robot state
    for sol in sols[::10]: # go through every 10th solution
        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        env.UpdatePublishedBodies() # allow viewer to update new robot
        raw_input('press any key')

raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original

# quat_target = [1,0,0,0] # wxyz
# #xyz_target = [6.51073449e-01,  -1.87673551e-01, 4.91061915e-01]
# xyz_target = [9.41799745e-01, -1.88000000e-01, 7.94677386e-01]

# #xyz_target = [6.51073449e-01,  -1.87673551e-01, -2.91061915e-01]
# hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

# # generate the ik solver
# ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
# if not ikmodel.load():
# 	ikmodel.autogenerate()
# with env:
#     while True:
#     	if not robot.CheckSelfCollision():
#         	solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetTransform(),IkFilterOptions.CheckEnvCollisions)
#         	print solutions
#         if((solutions is not None) and (len(solutions) > 0)):
#         	break 
            
    
#     print 'found %d solutions, rendering solutions:'%len(solutions)
#     # if len(solutions) < 10:
#     #     inds = range(len(solutions))
#     # else:
#     #     inds = array(linspace(0,len(solutions)-1,10),int)
#     # for i,ind in enumerate(inds):
#     #     print ind
#     #     newrobot = newrobots[i]
#     #     env.Add(newrobot,True)
#     #     newrobot.SetTransform(robot.GetTransform())
#     robot.SetDOFValues(solutions[0],ikmodel.manip.GetArmIndices())

# env.UpdatePublishedBodies()
# print('waiting...')
# time.sleep(20)




##NOTES##
###Generate all ik solutions (with identity as rotation matrix): ideal traj
#and then optimize::

#(1) generate all ik solutions with identity as rotation matrix as ideal trajectory 
#and then optimize


# reach_trans = array([[ 1.        ,  0.        ,  0.        ,  0.55308741],
#        [ 0.        ,  1.        ,  0.        , -0.42969778],
#        [ 0.        ,  0.        ,  1.        ,  0.82890214],
#        [ 0.        ,  0.        ,  0.        ,  1.        ]])

# manip.gettransform (proper reach)
#  = array([[ 0.58812355, -0.8057815 ,  0.06947568,  0.55308741],
#        [-0.00664476,  0.0810858 ,  0.99668498, -0.42969778],
#        [-0.8087438 , -0.58663556,  0.04233424,  0.82890214],
#        [ 0.        ,  0.        ,  0.        ,  1.        ]])

#  And then I can take these two to create solutions and then try out the diff solutions


#(2) generate an ideal trajectory with the contraint that I have to move as many joints as possible 



##END OF NOTES##



# newrobots = []
# for ind in range(10):
#     newrobot = openravepy.RaveCreateRobot(env,robot.GetXMLId())
#     newrobot.Clone(robot,0)
#     for link in newrobot.GetLinks():
#         for geom in link.GetGeometries():
#             geom.SetTransparency(0.4)
#     newrobots.append(newrobot)
# for link in robot.GetLinks():
#     for geom in link.GetGeometries():
#         geom.SetTransparency(0.4)

# while True:
#     # generate the ik solver
#     ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
#     if not ikmodel.load():
#         ikmodel.autogenerate()
#     with env:
#         # move the robot in a random collision-free position and call the IK
#         while True:
#             lower,upper = [v[ikmodel.manip.GetArmIndices()] for v in ikmodel.robot.GetDOFLimits()]
#             robot.SetDOFValues(np.random.rand()*(upper-lower)+lower,ikmodel.manip.GetArmIndices()) # set random values
#             if not robot.CheckSelfCollision():
#                 solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetTransform(),IkFilterOptions.CheckEnvCollisions)
#                 if solutions is not None and len(solutions) > 0: # if found, then break
#                     break
        
#         print 'found %d solutions, rendering solutions:'%len(solutions)
#         # if len(solutions) < 10:
#         #     inds = range(len(solutions))
#         # else:
#         #     inds = array(linspace(0,len(solutions)-1,10),int)
#         # for i,ind in enumerate(inds):
#         #     print ind
#         #     newrobot = newrobots[i]
#         #     env.Add(newrobot,True)
#         #     newrobot.SetTransform(robot.GetTransform())
#         #     newrobot.SetDOFValues(solutions[ind],ikmodel.manip.GetArmIndices())

#     env.UpdatePublishedBodies()
#     print('waiting...')
#     time.sleep(20)
    # remove the robots
#     for newrobot in newrobots:
#         env.Remove(newrobot)
# del newrobots