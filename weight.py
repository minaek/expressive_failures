import openravepy
from prpy.bind import bind_subclass
from archierobot import ArchieRobot
from catkin.find_in_workspaces import find_in_workspaces
import numpy as np
import logging

# Silence the planning logger to prevent spam.
logging.getLogger('prpy.planning.base').addHandler(logging.NullHandler())

robot_starting_dofs = np.array([-1, 2, 0, 2, 0, 4, 0, 1.11022302e-16,  -1.11022302e-16, 3.33066907e-16])

def initialize():
    '''Load and configure the Archie robot. Returns robot and environment.'''
    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    # Assumes the robot files are located in the urdf folder of the
    # kinova_description package in the catkin workspace.
    urdf_uri = 'package://interactpy/jaco.urdf'
    srdf_uri = 'package://interactpy/jaco.srdf'
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
    env.Load('{:s}/table.xml'.format(objects_path))
    env.Load('{:s}/mug1.kinbody.xml'.format(objects_path))
    table = env.GetKinBody('table')
    table.SetTransform(np.array([[1.00000000e+00, -2.79931237e-36,  1.41282351e-14, 1.50902510e-01],
                                 [-2.07944729e-28,  1.00000000e+00,  1.47183799e-14, -1.47385532e-02],
                                 [-1.41282351e-14, -1.47183799e-14,  1.00000000e+00, -1.00134850e-01],
                                 [0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]]))
    mug = env.GetKinBody('mug')
    mug.SetTransform(np.array([[1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 6.57676935e-01],
                               [0.00000000e+00, 0.00000000e+00, -1.00000000e+00, -3.07691471e-06],
                               [0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 5.46909690e-01],
                               [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]))

    # TODO(allanzhou): Decide if this is really necessary.
    robot.SetActiveDOFs(np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]))
    robot.SetDOFValues(robot_starting_dofs)
    return env, robot

def demo(robot):
    robot.SetDOFValues(robot_starting_dofs)
    orig_ee = robot.arm.hand.GetTransform()
    goal = np.array([0,  2.9 ,  0.0 ,  2.1 ,  0. ,  4. ,  0.])
    path = robot.arm.PlanToConfiguration(goal)
    robot.ExecutePath(path)
    goal2 = np.array([0,  2.9 ,  4.0 ,  2.1 ,  0. ,  4. ,  0.])
    path2 = robot.arm.PlanToConfiguration(goal2)
    robot.ExecutePath(path2)
    #path_back = robot.arm.PlanToEndEffectorPose(orig_ee)
    #robot.ExecutePath(path_back)