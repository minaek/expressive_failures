import openravepy
from openravepy import *
import numpy as np, math

"""Initializes all the global variables"""



#LIFTING_GOAL = [8.54594490e-01, -1.88000000e-01,  1.19776467e+00]

Final_pose = np.array([-0.20907978, -0.35360022, -0.03906491, -1.24079703,  3.36408011,
     -1.58597214, -3.10058622])


#ideal config for bent elbow starting config 
#ideal_config = [-0.08887252, -0.33820952, -0.05287741, -1.1905393 ,  2.85026761,
#       -1.07215964, -2.58677372]

# ideal_config = [-1.        , -0.5       , -1.3       , -1.7       ,  0.1       ,
#         0.09999996, -1.5       ] #original

final_lift = [-1.0, -1, -1, -1.7, 1.7, 0, 0]

ideal_config_vanilla = [-0.59435444, -0.35360022, -0.70886542, -1.04879236,  0.03297965,
       -0.10000004, -1.5       ]

final_pull = [-0.31004544, -0.10073148, -0.5       , -1.81371156, -2.90582601,
       -1.34329859, -0.26729975]

#ideal_config = final_lift

#Set up environment 
env = Environment()
env.SetViewer('qtcoin')
env.StopSimulation()


#arim position for lever#[-0.5       ,  0.        , -1.        , -1.7       , -1.6       ,
#       -1.34329859,  0.5       ]


###FOR LIFTING####
robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
env.AddRobot(robot)
target = env.ReadKinBodyXMLFile('/home/viki/openrave/src/data/mug1.kinbody.xml')
#env.AddKinBody(target)
table = env.ReadKinBodyXMLFile('/home/viki/openrave/src/data/table.kinbody.xml')
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

# table.SetTransform(
# 	   [[ 0.   , -1.   ,  0.   ,  0.7  ],
#        [ 1.   ,  0.   ,  0.   ,  0.   ],
#        [ 0.   ,  0.   ,  1.   ,  0.67],
#        [ 0.   ,  0.   ,  0.   ,  1.   ]])


#for joint_works
table.SetTransform([[ 0.   , -1.   ,  0.   ,  0.755],
       [ 1.   ,  0.   ,  0.   ,  0.   ],
       [ 0.   ,  0.   ,  1.   ,  0.689],
       [ 0.   ,  0.   ,  0.   ,  1.   ]])
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
target.SetTransform([[  3.88578059e-16,  -2.22044605e-16,   1.00000000e+00,
          5.50000000e-01],
       [  1.00000000e+00,  -5.55111512e-17,  -3.88578059e-16,
         -3.20000000e-01],
       [  5.55111512e-17,   1.00000000e+00,   2.22044605e-16,
          7.37000000e-01],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])


####FOR LEVER####
# env.Load('data/pr2test1.env.xml')
# target = env.GetKinBody('lever')
# if target is None:
#     target = RaveCreateKinBody(env,'')
#     target.InitFromBoxes(np.array([[0,0.1,0,0.01,0.1,0.01]]),True)
#     target.SetName('lever')
#     env.Add(target)
#     T = np.eye(4)
#     T[0:3,3] = [-0.2,-0.2,1]
#     target.SetTransform(T)


# robot = env.GetRobots()[0]




#robot = env.GetRobots()[0] #define robot 
manip_left = robot.GetManipulator("leftarm")
#robot.SetActiveManipulator(manip_left)
#left_arm_dofs = [ 1.  , -0.05,  2.  , -1.5 ,  1.  , -1.  ,  0.  ]

#left arm for joint_start_works
left_arm_dofs = [1.0, -0.05, 1, -1.5, 1.0, -1.0, 0.0]
robot.SetDOFValues(left_arm_dofs, manip_left.GetArmIndices())
manip = robot.GetManipulator("rightarm")

robot.SetActiveManipulator(manip)
manip_name = "rightarm"
ee_link_name = "r_gripper_tool_frame"
ee_link = robot.GetLink(ee_link_name)

n_steps = 10



##start config##
#joint_start = [-1. ,  0. , -1.5, -1.7,  0. ,  0. , -1.5] #this is for the bent arm 
#joint_start = [-1. ,  0. , -1.5, -1.7,  1.7,  0. ,  0. ]
joint_start_works = [-0.96312055,  0.41711733, -1.23801585, -1.70895655, -1.68348508,
       -0.17654026, -3.58644925]
js_higher = [-0.97841667,  0.35796416, -1.31326899, -1.70530046, -1.69949935,
       -0.10000717, -3.5190938 ]
joint_start_gazebo = [-0.34912021,  0.21717035, -1.9241912 , -1.20652939,  1.1320245 ,
       -0.42645212,  0.32148601]
# jsg_low = [-0.3362314 ,  0.64878883, -1.14855121, -1.20951067, -0.98957128,
#        -0.37490057, -4.45845703]
jsg_low = [-0.3362314 ,  0.64878883, -1.14855121, -1.20951067, -2.        ,
       -0.1       , -0.4       ]
gazebo2 = [-0.34122507,  0.63520348, -1.17509977, -1.20631928, -0.93695992, -0.36576328, -4.49143664]
narrower = [-0.5       ,  0.1       , -1.4       , -1.2       , -2.        ,
       -0.10000004, -0.4       ]
narrower_and_higher = [-0.50452918, -0.05452742, -1.66842242, -1.19887867,  1.99364131,
       -0.10204902,  2.13487526]
# jsg_low2 = [-0.3362314 ,  0.64878883, -1.14855121, -1.2095    ,  1.3       ,
#        -0.10000004, -0.4       ]
pull_start = [-0.31004544, -0.10073148, -1.27499825, -1.81371156, -2.90582601, -1.34329859,
 -0.26729975]

# lower,upper = robot.GetDOFLimits(manip.GetArmIndices()) # get the limits of just the arm indices #random
# robot.SetDOFValues(lower+numpy.random.rand(len(lower))*(upper-lower),manip.GetArmIndices())

Tgoal = [[ 0.25367689, -0.57112269,  0.78068362,  0.60407119],
       [-0.02689926,  0.80260678,  0.59590166, -0.19286113],
       [-0.96691492, -0.17216629,  0.18824001,  0.98687443],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]


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
    sols = manip.FindIKSolutions(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    return sol,sols


#initializing the list of solutions that best_starting_config() in trajprac2.py will use
_,sols = iksolver(Tgoal)
starting_configs = []
starting_transforms = []
goal_configs=[]
for sol in sols:
	starting_configs.append(sol)
	robot.SetDOFValues(sol, manip.GetArmIndices())
	starting_transforms.append(manip.GetEndEffectorTransform())
	goal_transform = manip.GetEndEffectorTransform()
	goal_transform[:3][:,3][2] +=0.3
	goal_config,_ = iksolver(goal_transform)
	goal_configs.append(goal_config)

# robot.SetDOFValues(js_higher, manip.GetArmIndices())
# starting_config = manip.GetArmDOFValues()
# starting_transform = manip.GetEndEffectorTransform()


# goal_config = iksolver(ideal_config).tolist()
# goal_config = [-0.8678595041624291,
#  -0.16081990854357753,
#  -0.7485512099999999,
#  -1.7251806369682334,
#  -1.679464146676161,
#  -0.21332417643351156,
#  0.04771217062680844]


# def iksolver(Tgoal):
#     """
#     Returns a goal configuration using the IK solver. 
#     We do not optimize for which configuration is returned yet.
#     ---
#     output: configuration
#     """
#     global robot
#     # generate the ik solver
#     manip = robot.SetActiveManipulator('rightarm')
#     robot.SetActiveManipulator(manip)
#     ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
#     if not ikmodel.load():
#         ikmodel.autogenerate()
#     # with robot:
#     #     robot.SetDOFValues(ideal_config, manip.GetArmIndices())
#     #     Tgoal = manip.GetEndEffectorTransform()
#     sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
#     robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
#     return sol


#starting_transform[:3][:,3][2] +=0.3
#goal_config = iksolver(starting_transform)
#robot.SetDOFValues(js_higher, manip.GetArmIndices())

#open gripper
robot.SetDOFValues([1],[34])


