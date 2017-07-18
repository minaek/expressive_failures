from demo import *
from trajprac2 import *
import pr2_util
import roslib
roslib.load_manifest("control_msgs")
import rospy
import trajoptpy
from catkin.find_in_workspaces import find_in_workspaces
import commands, logging, math, numpy as np, os, time
from openravepy import *
from lfd.rapprentice import PR2
from lfd.environment.simulation import DynamicSimulationRobotWorld
from lfd.environment.simulation_object import XmlSimulationObject, \
        TableSimulationObject, BoxSimulationObject
from lfd.demonstration import demonstration
from lfd.util import util



def init():
	global or_sim, pr2, v, cup, ideal_config, goal_config, starting_config
	print "initializing rospy node.."
	rospy.init_node("manip_task",disable_signals=True)

	print "initializing scene.."
	or_sim, pr2, v, cup = initialize(load_gazebo=LOAD_GAZEBO)

	print "solving for goal configuration.."
	ideal_config = [-1.0, -1, -1, -1.7, 1.7, 0, 0]
	robot = or_sim.env.GetRobots()[0]
	# generate the ik solver
	manip = robot.SetActiveManipulator('rightarm')
	robot.SetActiveManipulator(manip)
	ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
	if not ikmodel.load():
	    ikmodel.autogenerate()
	with robot:
	    robot.SetDOFValues(ideal_config, manip.GetArmIndices())
	    Tgoal = manip.GetEndEffectorTransform()
	goal_config = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
	#robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
	goal_config = goal_config.tolist()

	print "grasping cup.."
	starting_config = grasp_cup(or_sim, pr2, cup) #this is the trajectory that graspss the cup
	# starting_config = np.array([-0.34912021,  0.21717035, -1.9241912 , -1.20652939,  1.1320245 ,
 #       -0.42645212,  0.32148601])
	# update_or_robot(robot, starting_config, rave_inds=manip.GetArmIndices())

