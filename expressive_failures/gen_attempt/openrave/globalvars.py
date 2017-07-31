import openravepy
from openravepy import *
import numpy as np, math

"""Initializes all the global variables"""


#Set up environment 
env = Environment()
env.SetViewer('qtcoin')
env.StopSimulation()


#arm position for lever#[-0.5       ,  0.        , -1.        , -1.7       , -1.6       ,
#       -1.34329859,  0.5       ]


###FOR LIFTING####
robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
env.AddRobot(robot)
#target = env.ReadKinBodyXMLFile('sphere.xml')
#target = env.ReadKinBodyXMLFile('/home/viki/openrave/src/data/mug1.kinbody.xml')
#env.AddKinBody(target)
# table = env.ReadKinBodyXMLFile('/home/shhuang/src/openrave/src/data/table.kinbody.xml')
#env.AddKinBody(table)

viewer = env.GetViewer()
viewer.SetCamera(
  np.array([[ -9.94542311e-03,   5.43414497e-01,  -8.39405607e-01,
          2.32253075e+00],
       [  9.99869989e-01,   1.60593593e-02,  -1.45012309e-03,
          3.47638549e-03],
       [  1.26922983e-02,  -8.39310897e-01,  -5.43503564e-01,
          1.97211254e+00],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]]))
# target.SetTransform([[  2.22044605e-16,  -4.44089210e-16,   1.00000000e+00,
#           6.60000000e-01],
#        [  1.00000000e+00,   9.86076132e-32,  -2.22044605e-16,
#           0.00000000e+00],
#        [  0.00000000e+00,   1.00000000e+00,   4.44089210e-16,
#           5.60000000e-01],
#        [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
#           1.00000000e+00]])
# table.SetTransform(
# 	   [[ 0.   , -1.   ,  0.   ,  0.7  ],
#        [ 1.   ,  0.   ,  0.   ,  0.   ],
#        [ 0.   ,  0.   ,  1.   ,  0.67],
#        [ 0.   ,  0.   ,  0.   ,  1.   ]])


#for joint_works
# table.SetTransform([[ 0.   , -1.   ,  0.   ,  0.755],
#        [ 1.   ,  0.   ,  0.   ,  0.   ],
#        [ 0.   ,  0.   ,  1.   ,  0.689],
#        [ 0.   ,  0.   ,  0.   ,  1.   ]])

#for joint_start gazebo
# target.SetTransform([[  2.22044605e-16,  -4.44089210e-16,   1.00000000e+00,
#           6.60000000e-01],
#        [  1.00000000e+00,   9.86076132e-32,  -2.22044605e-16,
#           0.00000000e+00],
#        [  0.00000000e+00,   1.00000000e+00,   4.44089210e-16,
#           5.60000000e-01],
#        [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
#           1.00000000e+00]])

#for jsg_low
# target.SetTransform([[  3.33066907e-16,  -3.88578059e-16,   1.00000000e+00,
#           7.25000000e-01],
#        [  1.00000000e+00,   0.00000000e+00,  -3.88578059e-16,
#          -1.50000000e-02],
#        [  0.00000000e+00,   1.00000000e+00,   3.33066907e-16,
#           5.20000000e-01],
#        [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
#           1.00000000e+00]])

#for joint_start_works
# target.SetTransform([[  3.88578059e-16,  -2.22044605e-16,   1.00000000e+00,
#           5.50000000e-01],
#        [  1.00000000e+00,  -5.55111512e-17,  -3.88578059e-16,
#          -3.20000000e-01],
#        [  5.55111512e-17,   1.00000000e+00,   2.22044605e-16,
#           7.37000000e-01],
#        [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
#           1.00000000e+00]])


####FOR LEVER####
# env.Load('/home/viki/catkin_ws/src/expressive_failures/expressive_failures/gen_attempt/openrave/pr2test1.env.xml')
# target = env.GetKinBody('lever')
# robot = env.GetRobots()[0]
# if target is None:
# 	target = RaveCreateKinBody(env,'')
# 	target.InitFromBoxes(np.array([[0,0.1,0,0.01,0.1,0.01]]),True)
# 	target.SetName('lever')
# 	env.Add(target)
# 	T = np.eye(4)
# 	T[0:3,3] = [-0.2,-0.2,1]
# 	target.SetTransform(T)



manip_left = robot.GetManipulator("leftarm")
left_arm_dofs = [1.0, -0.05, 1, -1.5, 1.0, -1.0, 0.0] #left arm dofs for joint_start_works
robot.SetDOFValues(left_arm_dofs, manip_left.GetArmIndices())
manip = robot.GetManipulator("rightarm")
robot.SetActiveManipulator(manip)
manip_name = "rightarm"
# ee_link_name = "r_gripper_tool_frame"
# ee_link = robot.GetLink(ee_link_name)
n_steps = 10


##start configs##
#For lifting
joint_start = [-1. ,  0. , -1.5, -1.7,  0. ,  0. , -1.5] #this is for the bent arm 
joint_start_works = [-0.96312055,  0.41711733, -1.23801585, -1.70895655, -1.68348508,
       -0.17654026, -3.58644925]
js_higher = [-0.97841667,  0.35796416, -1.31326899, -1.70530046, -1.69949935,
       -0.10000717, -3.5190938 ]
joint_start_gazebo = [-0.34912021,  0.21717035, -1.9241912 , -1.20652939,  1.1320245 ,
       -0.42645212,  0.32148601]
jsg_low = [-0.3362314 ,  0.64878883, -1.14855121, -1.20951067, -2.        ,
       -0.1       , -0.4       ]
gazebo2 = [-0.34122507,  0.63520348, -1.17509977, -1.20631928, -0.93695992, -0.36576328, -4.49143664]
narrower = [-0.5       ,  0.1       , -1.4       , -1.2       , -2.        ,
       -0.10000004, -0.4       ]
narrower_and_higher = [-0.50452918, -0.05452742, -1.66842242, -1.19887867,  1.99364131,
       -0.10204902,  2.13487526]
# jsg_low2 = [-0.3362314 ,  0.64878883, -1.14855121, -1.2095    ,  1.3       ,
#        -0.10000004, -0.4       ]

#For lever
grab_lever = [-0.02492772,  0.38241802, -0.58749861, -1.86802079,  3.13089202, -1.42422262,
 -1.02907588]

pull_start = [-0.31004544, -0.10073148, -1.27499825, -1.81371156, -2.90582601, -1.34329859,
 -0.26729975]

unscrewing = [-0.96312055,
 0.21711733000000008,
 -1.23801585,
 -2.10895655,
 -1.68348508,
 -1.6765402600000003,
 -4.38644925]

unscrewing2 = [-0.96312055,
 0.21711733000000008,
 -1.23801585,
 -2.25895655,
 0.01651491999999985,
 -0.37654026,
 -4.38644925] #doesn't work well because...???

unscrewing3 = [-0.96312055,
 0.71711733,
 -1.23801585,
 -2.25895655,
 1.4165149199999998,
 0.42345974,
 -3.2864492500000004]

unscrewing4 = [ 0.55251804,  0.44822124,  0.4       , -1.88039846,  2.98306223,
       -1.34365667, -0.19791446]

unscrewing5 = [-1.14748542,  0.44822124,  0.4       , -1.88039846,  2.98306223,
       -1.34365667, -0.19791446] #works, basically same thign as pull_lever

unscrewing6 = [-1.43216102,  0.50094937, -0.08076609, -1.89149012,  4.93991212,
       -1.39717667, -0.08199067] #works as well; desired for pushing forward
unscrewing65 = [-0.7321610199999999,
 0.50094937,
 -0.08076609,
 -0.8914901200000002,
 4.93991212,
 -0.9971766699999999,
 -1.58199067]


unscrewing7 = [-1.38381072,  0.50315089, -1.4       , -1.89197786, -1.76747669,
       -1.66354662, -0.0673271 ]

unscrewing_goal = [0.36312055,
 0.21711733000000008,
 -1.23801585,
 -1.70895655,
 -1.68348508,
 -1.6765402600000003,
 -5.186449249999997]


pushing = [0,0,0,0,0,0,0]
pulling = [0.0, 0.4, 0.0, -1.3, -1.4, 0.0, 0.0] #not successful

pushing_with_base = [-0.5,
 -0.20000000000000004,
 -1.4,
 -1.2,
 -1.7999999999999998,
 -0.10000004,
 -0.4]
#random initial positions
# lower,upper = robot.GetDOFLimits(manip.GetArmIndices()) # get the limits of just the arm indices #random
# robot.SetDOFValues(lower+numpy.random.rand(len(lower))*(upper-lower),manip.GetArmIndices())

# Tgoal = np.array([[ 0.25367689, -0.57112269,  0.78068362,  0.60407119],
#        [-0.02689926,  0.80260678,  0.59590166, -0.19286113],
#        [-0.96691492, -0.17216629,  0.18824001,  0.98687443],
#        [ 0.        ,  0.        ,  0.        ,  1.        ]])
# Tgoal[:3][:,3][0] += np.random.uniform(-0.2,0.2)
# Tgoal[:3][:,3][1] += np.random.uniform(-0.2,0.2)
# Tgoal[:3][:,3][2] += np.random.uniform(-0.2,0.2)

# Other option: set Tgoal based on end effector position of a particular starting config

starting_config = pushing_with_base
robot.SetDOFValues(starting_config, manip.GetArmIndices())
Tgoal = manip.GetEndEffectorTransform()

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
    sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    sols = manip.FindIKSolutions(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solutions
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



#initializing the list of solutions that best_starting_config() in trajprac2.py will use
#curr_base = robot.GetTransform()[:3][:,3]

# base_positions = [curr_base,[0.05,0,0],[-0.05,0,0],[0,0.05,0],[0,-0.05,0]] #five base positions
# total_sols=[]
# for base in base_positions:
# 	transform = robot.GetTransform() #get current transform
# 	transform[:3][:,3] = base #change base position 
# 	robot.SetTransform(transform) #move to new base position 
# 	_,arm_sols = iksolver(Tgoal)
# 	total_sols.append(arm_sols)

# goal_config=[]
# for s in total_sols:
# 	if len(s)!=0:
# 		robot.SetDOFValues(total_sols[0][0], manip.GetArmIndices())
# 		goal_transform = manip.GetEndEffectorTransform()
# 		goal_transform[:3][:,3][2] -= 0.2  # For lifting, should not make this too high, or else arm will stay still
# 		#goal_transform[:3][:,3][1] -= 0.2  # For pulling lever
# 		#goal_transform[:3][:,3][0] += 0.2  # For pulling lever
# 		goal_config,_ = iksolver(goal_transform)

# 		robot.SetDOFValues([1],[34]) #open gripper
# 		break
# 	else:
# 		print "No solution found."


# transform = robot.GetTransform() #get current transform
# transform[:3][:,3] = curr_base #change base position 
# robot.SetTransform(transform) 

###############################################################
#This is for optimizing for arm configs with non-discretized base positions. 
starting_configs = []
starting_transforms = []
_,sols = iksolver(Tgoal)
if len(sols)==0:
	print "No solution found."

else:
	#for all else:
	robot.SetDOFValues(sols[0], manip.GetArmIndices())
	goal_transform = manip.GetEndEffectorTransform()
	#goal_transform[:3][:,3][2] -= 0.2  # For lifting, should not make this too high, or else arm will stay still
	#goal_transform[:3][:,3][1] += 0.2  # For pulling lever
	goal_transform[:3][:,3][0] += 0.2  # For pulling lever
	goal_config_stationary,_ = iksolver(goal_transform)
	for sol in sols:
		starting_configs.append(sol)
		robot.SetDOFValues(sol, manip.GetArmIndices())
		#starting_transforms.append(manip.GetEndEffectorTransform())

	#open gripper
	robot.SetDOFValues([1],[34])


