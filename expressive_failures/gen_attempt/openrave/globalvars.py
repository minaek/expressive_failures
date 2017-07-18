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

ideal_config = [-1.0, -1, -1, -1.7, 1.7, 0, 0]


ideal_config_vanilla = [-0.59435444, -0.35360022, -0.70886542, -1.04879236,  0.03297965,
       -0.10000004, -1.5       ]


#Set up environment 
env = Environment()
env.SetViewer('qtcoin')
env.StopSimulation()
# env.Load('data/expressive.xml')
#env.Load("robots/pr2-beta-static.zae")
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

table.SetTransform([[ 0.  , -1.  ,  0.  ,  0.7 ],
       [ 1.  ,  0.  ,  0.  ,  0.  ],
       [ 0.  ,  0.  ,  1.  ,  0.65],
       [ 0.  ,  0.  ,  0.  ,  1.  ]])


target.SetTransform([[  1.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          6.00000000e-01],
       [  0.00000000e+00,   4.44089210e-16,  -1.00000000e+00,
         -2.00000000e-01],
       [  0.00000000e+00,   1.00000000e+00,   4.44089210e-16,
          6.70000000e-01],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])



#robot = env.GetRobots()[0] #define robot 
manip = robot.GetManipulator("rightarm")

robot.SetActiveManipulator(manip)
manip_name = "rightarm"
ee_link_name = "r_gripper_tool_frame"
ee_link = robot.GetLink(ee_link_name)

n_steps = 10

##Initialize robot start config##
#joint_start = [-1. ,  0. , -1.5, -1.7,  0. ,  0. , -1.5] #this is for the bent arm 
#joint_start = [-1. ,  0. , -1.5, -1.7,  1.7,  0. ,  0. ]
joint_start = [-1.        ,  0.2       , -1.5       , -1.7       ,  1.7       ,
       -0.10000004, -0.5       ]
joint_start_gazebo = [-0.34912021,  0.21717035, -1.9241912 , -1.20652939,  1.1320245 ,
       -0.42645212,  0.32148601]
robot.SetDOFValues(joint_start_gazebo, manip.GetArmIndices())

#joint_start = np.zeros(7) #straight arm
#robot.SetDOFValues(joint_start, manip.GetArmIndices())

# lower,upper = robot.GetDOFLimits(manip.GetArmIndices()) # get the limits of just the arm indices #random
# robot.SetDOFValues(lower+numpy.random.rand(len(lower))*(upper-lower),manip.GetArmIndices())

starting_config = manip.GetArmDOFValues()
starting_transform = manip.GetEndEffectorTransform()
ee_link_z = manip.GetEndEffectorTransform()[:3][:,3]
ee_link_z[2] = ee_link_z[2]+0.5
LIFTING_GOAL = list(ee_link_z)

def iksolver(ideal_config):
    """
    Returns a goal configuration using the IK solver. 
    We do not optimize for which configuration is returned yet.
    ---
    output: configuration
    """
    global robot
    # generate the ik solver
    manip = robot.SetActiveManipulator('rightarm')
    robot.SetActiveManipulator(manip)
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    with robot:
        robot.SetDOFValues(ideal_config, manip.GetArmIndices())
        Tgoal = manip.GetEndEffectorTransform()
    sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
    return sol

goal_config = iksolver(ideal_config).tolist()


##grasping##
