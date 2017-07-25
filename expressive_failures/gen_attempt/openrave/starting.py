from env import *
import numpy as np

Tgoal = [[ 0.25367689, -0.57112269,  0.78068362,  0.60407119],
       [-0.02689926,  0.80260678,  0.59590166, -0.29286113],
       [-0.96691492, -0.17216629,  0.18824001,  0.78687443],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]


def iksolver2(Tgoal):
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
    # with robot:
    #     robot.SetDOFValues(ideal_config, manip.GetArmIndices())
    #     Tgoal = manip.GetEndEffectorTransform()
    sol = manip.FindIKSolutions(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    return sol


sols = iksolver2(Tgoal)
