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



