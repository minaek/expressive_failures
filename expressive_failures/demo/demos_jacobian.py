import openravepy
from openravepy import *
from prpy.bind import bind_subclass
from archierobot import ArchieRobot
from catkin.find_in_workspaces import find_in_workspaces
import numpy as np
import logging
import math
from numpy.linalg import pinv
import time

# Silence the planning logger to prevent spam.
logging.getLogger('prpy.planning.base').addHandler(logging.NullHandler())

robot_starting_dofs = np.array([1, 2, 0, 2, 0, 4, 0, 1.11022302e-16,  -1.11022302e-16, 3.33066907e-16])

def initialize():
    '''Load and configure the Archie robot. Returns robot and environment.'''
    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    # Assumes the robot files are located in the urdf folder of the
    # kinova_description package in the catkin workspace.
    #env.Load('lab.xml')

    urdf_uri = 'package://interactpy/jaco_dynamics.urdf'
    srdf_uri = 'package://interactpy/jaco_dynamics.srdf'
    objects_path = find_in_workspaces(
        project='interactpy',
        path='envdata',
        first_match_only=True)[0]
    or_urdf = openravepy.RaveCreateModule(env, 'urdf')
    robot_name = or_urdf.SendCommand(
        'load {:s} {:s}'.format(urdf_uri, srdf_uri))

    robot = env.GetRobot(robot_name)
    bind_subclass(robot, ArchieRobot)
    # Put robot in natural starting config.
    # Load environment
    #env.Load('{:s}/table.xml'.format(objects_path))
    env.Load('{:s}/mug1.kinbody.xml'.format(objects_path))
    # table = env.GetKinBody('table')J
    # table.SetTransform(np.array([[1.00000000e+00, -2.79931237e-36,  1.41282351e-14, 1.50902510e-01],
    #                              [-2.07944729e-28,  1.00000000e+00,  1.47183799e-14, -1.47385532e-02],
    #                              [-1.41282351e-14, -1.47183799e-14,  1.00000000e+00, -1.00134850e-01],
    #                              [0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]]))
    mug = env.GetKinBody('mug')
    # mug.SetTransform(np.array([[1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 6.57676935e-01],
    #                            [0.00000000e+00, 0.00000000e+00, -1.00000000e+00, -3.07691471e-06],
    #                            [0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 5.46909690e-01],
    #                            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]))
   
    #mug.SetTransform(np.array([[1.00000000e+00, 0.00000000e+00, 0.00000000e+00, -3.00000000e-01],
                                #   [0.00000000e+00, 0.00000000e+00, -1.00000000e+00, 1.00000000e+00],
                                #   [0.00000000e+00, 1.00000000e+00, 0.00000000e+00, -1.00000000e-01],
                                #   [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]))
    # mug.SetTransform(np.array([[1.00000000e+00, 0.00000000e+00, 0.00000000e+00, -0.10000000],
    #                               [0.00000000e+00, 0.00000000e+00, -1.00000000e+00, 0.20000000e+00],
    #                               [0.00000000e+00, 1.00000000e+00, 0.00000000e+00, -1.00000000e-01],
    #                               [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]))

    
    mug.SetTransform([[1,0,0,-0.4],[0,0,-1,0.3],[0,1,0,-0.1]])

    #mug.SetTransform([[1,0,0,-0.6],[0,0,-1,-0.6],[0,1,0,.7]])
    robot.SetActiveDOFs(np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]))
    robot.SetDOFValues(robot_starting_dofs)
    return env,robot,mug


def JacPICtrl(env,robot,mug):
    DOFs = []

    # viewer = env.GetViewer()

    # viewer.SetCamera([[ 0.94684722, -0.12076704,  0.29815376,  0.21004671],
    #    [-0.3208323 , -0.42191214,  0.84797216, -1.40675116],
    #    [ 0.0233876 , -0.89855744, -0.43823231,  1.04685986],
    #    [ 0.        ,  0.        ,  0.        ,  1.        ]])
    mug.SetTransform([[1,0,0,-0.4],[0,0,-1,0.3],[0,1,0,-0.1]])
    D = mug.GetTransform()
    Dvec = D[:,3][0:-1]
    Rvec = robot.GetActiveManipulator().GetEndEffectorTransform()[:,3][0:-1]
    physics = RaveCreatePhysicsEngine(env,'ode')
    env.SetPhysicsEngine(physics)
    physics.SetGravity(np.array((0,0,0)))
    while((np.abs(Rvec-Dvec)>np.array([0.0001,0.0001,0.0001])).all()):
        base = robot.GetLinks()[8]
        hand = robot.GetLinks()[7]
        finger = robot.GetLinks()[10]
        if env.CheckCollision(robot,mug)==True:
            break

        elif env.CheckCollision(hand,base)==True:
            break
        else:
            currDOFs = robot.GetActiveDOFValues()
            Rvec = robot.GetActiveManipulator().GetEndEffectorTransform()[:,3][0:-1]
            deltaX = (Dvec - Rvec)*0.001
            J = robot.GetActiveManipulator().CalculateJacobian()
            Jplus = J.transpose().dot(pinv(J.dot(J.transpose())))
            newDOFs = currDOFs + (np.append(Jplus.dot(deltaX),np.zeros(3)))
            DOFs.append(newDOFs)
            robot.SetDOFValues(newDOFs)
    playback(DOFs)
    print "done"

def playback(DOFs):
    playback_dofs= DOFs[480:]
    for _ in range(4):
        for dof in reversed(playback_dofs):
            robot.SetDOFValues(dof)
            time.sleep(0.005)
        for dof in playback_dofs:
            robot.SetDOFValues(dof)
            time.sleep(0.005)

def reach(env,robot,mug):
    DOFs = []

    viewer = env.GetViewer()

    viewer.SetCamera([[ 0.94684722, -0.12076704,  0.29815376,  0.21004671],
       [-0.3208323 , -0.42191214,  0.84797216, -1.40675116],
       [ 0.0233876 , -0.89855744, -0.43823231,  1.04685986],
       [ 0.        ,  0.        ,  0.        ,  1.        ]], -1)
    mug.SetTransform([[1,0,0,0.7],[0,0,-1,-.8],[0,1,0,.5]])


    D = mug.GetTransform()
    Dvec = D[:,3][0:-1]
    Rvec = robot.GetActiveManipulator().GetEndEffectorTransform()[:,3][0:-1]
    physics = RaveCreatePhysicsEngine(env,'ode')
    env.SetPhysicsEngine(physics)
    physics.SetGravity(np.array((0,0,0)))
    while(np.sum((np.abs(Rvec-Dvec)>np.array([0.2,0.2,0.2])))>0):
        currDOFs = robot.GetActiveDOFValues()
        Rvec = robot.GetActiveManipulator().GetEndEffectorTransform()[:,3][0:-1]
        deltaX = (Dvec - Rvec)*0.0088
        J = robot.GetActiveManipulator().CalculateJacobian()
        Jt = J.transpose()
        newDOFs = currDOFs + (np.append(Jt.dot(deltaX),np.zeros(3)))
        DOFs.append(newDOFs)
        robot.SetDOFValues(newDOFs)

    playback_reach(DOFs)
    print "done"


def playback_reach(DOFs):
    playback_dofs= DOFs[1800:]
    for _ in range(4):
        for dof in reversed(playback_dofs):
            robot.SetDOFValues(dof)
            time.sleep(0.0001)
        for dof in playback_dofs:
            robot.SetDOFValues(dof)
            time.sleep(0.0001)

# def physics():
#     env, robot,mug = initialize()

#     physics = RaveCreatePhysicsEngine(env,'ode')
#     env.SetPhysicsEngine(physics)
#     physics.SetGravity(np.array((0,0,0))) #should be (0,0,-9.8)

#     #env.StopSimulation()
#     #env.StartSimulation(timestep=0.001)

#     robot.SetActiveDOFs(np.array([0, 1, 2, 3, 4, 5, 6]))
#     viewer = env.GetViewer()
#     viewer.SetCamera([[ 0.94684722, -0.12076704,  0.29815376,  0.21004671],
#            [-0.3208323 , -0.42191214,  0.84797216, -1.40675116],
#            [ 0.0233876 , -0.89855744, -0.43823231,  1.04685986],
#            [ 0.        ,  0.        ,  0.        ,  1.        ]])
#     for _ in range(10):
#         #torques = compute_idyn(robot)
#         torques = np.array([0,0,0,0,0,0,20,0,0,0]) #100*(numpy.random.rand(robot.GetDOF())-0.5)
#         print torques
#         for i in range(10):
#             robot.SetJointTorques(torques,True)
#             time.sleep(0.01)
#     return env,robot

#possible dof value array:
#([ -2.07528844e+02,   5.09627363e+01,   1.13966137e+02,
#        -5.04338173e+02,  -3.44717320e+01,   9.74304478e+02,
#         2.56020831e-14,   1.26565425e-13,   2.22044605e-16,
#         6.21724894e-15])





# def sim(env,robot):
#     robot.SetDOFValues(robot_starting_dofs)
#     orig_ee = robot.arm.hand.GetTransform()
#     goal = row_to_column("jointlimit.csv")


#     goal[:,2] = goal[:,2] + math.pi #adding pi to the third joint (indexed from 1)
#     robot.SetDOFValues(np.append(goal[0],np.zeros(3))) #adding 3 more dofs for the fingers
#     executePathSim(env,robot,goal)
 

# def executePathSim(env,robot,waypts):
#     """
#     Executes in the planned trajectory in simulation
#     """

#     traj = RaveCreateTrajectory(env,'')
#     traj.Init(robot.GetActiveConfigurationSpecification())
#     for i in range(len(waypts)):
#         traj.Insert(i, np.append(waypts[i], np.zeros(3)))
#     robot.ExecutePath(traj)