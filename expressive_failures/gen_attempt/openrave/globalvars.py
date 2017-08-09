"""Initializes all the global variables"""

import openravepy
from openravepy import *
import numpy as np, math
import params

def iksolver(Tgoal):
    """
    Returns a goal configuration and list of configruations using the IK solver. 
    We do not optimize for which configuration is returned yet.
    ---
    output: configuration and list of configurations
    """
    global robot
    # generate the ik solver
    manip = robot.SetActiveManipulator('rightarm')
    robot.SetActiveManipulator(manip)
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    # with robot:
    #     robot.SetDOFValues(ideal_config, manip.GetArmIndices())
    #     Tgoal = manip.GetEndEffectorTransform()
    #sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    sol = manip.FindIKSolution(Tgoal, IkFilterOptions.IgnoreEndEffectorCollisions)
    sols = manip.FindIKSolutions(Tgoal, IkFilterOptions.IgnoreEndEffectorCollisions) # get collision-free solutions
    return sol,sols

def gen_rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

#Set up environment 
env = Environment()
env.SetViewer('qtcoin')
env.StopSimulation()
robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
env.AddRobot(robot)

manip_left = robot.GetManipulator("leftarm")
robot.SetDOFValues(params.left_arm_attitude, manip_left.GetArmIndices())
manip = robot.GetManipulator("rightarm")
robot.SetActiveManipulator(manip)
manip_name = "rightarm"

#########CHANGE CODE HERE TO PREFERRED TASK##########
TASK = params.PUSH_SIDEWAYS #define task 
#########CHANGE CODE HERE TO PREFERRED TASK##########

robot.SetDOFValues(TASK["default_starting_config"], manip.GetArmIndices()) #set dof value to starting config

viewer = env.GetViewer()
viewer.SetCamera(TASK["camera_array"])


for t in range(len(TASK["target_transform"])):
  target = env.ReadKinBodyXMLFile(TASK["target"][t])
  env.AddKinBody(target) #add target
  target.SetTransform(TASK["target_transform"][t])#set target transform

for p in range(len(TASK["props"])): 
  prop = env.ReadKinBodyXMLFile(TASK["props"][p])
  env.AddKinBody(prop) #add props
  prop.SetTransform(TASK["props_transform"][p]) #set prop trasnforms

#update Tgoal in dictionary 
if TASK == params.LIFT:
  Tgoal = manip.GetEndEffectorTransform()
  Tgoal[:3][:,3][:2] = target.GetTransform()[:3][:,3][:2]
  TASK["Tgoal"] = Tgoal

else:
  robot.SetDOFValues(TASK["default_starting_config"], manip.GetArmIndices())
  Tgoal = manip.GetEndEffectorTransform()
  TASK["Tgoal"] = Tgoal


#use ik solver to solve for various starting configs and a goal config 
starting_configs = []
starting_transforms = []
_,sols = iksolver(Tgoal)

if len(sols)==0: #there are no viable sollutions for the starting xyz position 
	print "No solution found."
else:
  robot.SetDOFValues(sols[0], manip.GetArmIndices()) #set dofs to first ik solution found; doesn't matter which ik solution we use
  goal_transform = manip.GetEndEffectorTransform()
  axis = TASK["axis"]
  vector = TASK["vector"]
  goal_transform[:3][:,3][axis] += vector  # update goal_transform accordingly

  goal_config_stationary,_ = iksolver(goal_transform) #get ik solution for goal_transform
  if goal_config_stationary is None: #there are no viable solutions for goal_transform
	  print "No goal solution found."
  else:
	  robot.SetDOFValues(goal_config_stationary, manip.GetArmIndices())
	  tr = manip.GetEndEffectorTransform()
	  assert (goal_transform-tr<1e-10).all() #make sure this ik solution's xyz position is approximately equal to goal_transform's xyz
	  for sol in sols: #make sure the xyz positions for tall the starting position ik solutions are approximately equal to Tgoal' xyz
		  robot.SetDOFValues(sol, manip.GetArmIndices())
		  trans = manip.GetEndEffectorTransform()
		  assert (trans-Tgoal < 1e-9).all()
		  starting_configs.append(sol)

		#open gripper
	  robot.SetDOFValues([0],[34]) #set default gripper to closed 


