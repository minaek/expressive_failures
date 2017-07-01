import openravepy
from parse_csv import row_to_column
from openravepy import *
import numpy as np
import time


# robot_starting_dofs = np.array([0.0,0.0])
# def initialize():
#     env = openravepy.Environment()
#     env.Load('env.xml')
#     env.SetViewer('qtcoin')
#     robot = env.GetRobots()[0]
#     robot.SetDOFValues(robot_starting_dofs)

#     return env, robot

# def sim(env,robot):
#     robot.SetDOFValues(robot_starting_dofs)

#     goal = np.array([1.,0.])
#     path = robot.PlanToConfiguration(goal)
#     robot.ExecutePath(path)    
#     #executePathSim(env,robot,goal)
 

# def executePathSim(env,robot,waypts):
#     """
#     Executes in the planned trajectory in simulation
#     """

#     traj = RaveCreateTrajectory(env,'')
#     traj.Init(robot.GetActiveConfigurationSpecification())
#     for i in range(len(waypts)):
#         traj.Insert(i, waypts[i])
#     robot.ExecutePath(traj)

def initialize():
    env=Environment()
    env.Load('env.xml')
    env.SetViewer('qtcoin')
    with env:
        # set a physics engine
        physics = RaveCreatePhysicsEngine(env,'ode')
        env.SetPhysicsEngine(physics)
        physics.SetGravity(numpy.array((0,0,-9.8)))

        body = env.GetBodies()[0]
        robot = env.GetRobots()[0]
        robot.GetLinks()[0].SetStatic(True)

        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    return env, robot


def send_torques(env,robot):
    torques = 100*(numpy.random.rand(robot.GetDOF())-0.5)
    for i in range(100):
        with env:
            robot.SetDOFTorques(torques,True)
        time.sleep(0.1)
    env.SetPhysicsEngine(None)


# while True:
#     torques = 100*(numpy.random.rand(robot.GetDOF())-0.5)
#     for i in range(100):
#         robot.SetJointTorques(torques,True)
#         time.sleep(0.01)