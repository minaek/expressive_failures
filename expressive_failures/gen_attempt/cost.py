import globalvars
from globalvars import *
import numpy as np, math
from math import *


"""All cost functions"""

# manip = globalvars.manip
# starting_transform=globalvars.starting_transform
# robot = globalvars.robot
# ideal_config = globalvars.ideal_config

def features_quat(waypt):
    """
    Computes the distance from current waypoint
    to ideal waypoint using DOFs, but might change to coord.
    ---
    input trajectory, output scalar feature
    """
    global manip, starting_transform
    quat_target = list(transform2quat(manip.GetEndEffectorTransform()))
    quat_diff = sum(quat_target - starting_transform)
    return quat_dff

def cost_quat(waypt):
    """
    Computes the total distance between waypt and ideal waypt
    ---
    input trajectory, output scalar cost
    """
    feature = features_distance(waypt)
    #return feature*self.weights
    return feature

def features_largest_change(waypt, joint_num):
    """
    Computes the total cost over waypoints based on 
    z-axis distance to table
    ---
    input trajectory, output scalar feature
    """
    global robot,manip
    robot.SetDOFValues(waypt,manip.GetArmIndices())
    link_transformations = np.array(robot.GetLinkTransformations())

    link_trans = link_transformations[joint_num]
    z = link_trans[2][-1]
    return z
    # arm_inds = manip.GetArmIndices()
    # arm_transformations = link_transformations[arm_inds]
    # arm_z =[]
    # for trans in arm_transformations:
    #     arm_z.append(trans[2][-1])
    # sum_z = sum(arm_z)
    
    # return sum_z


def features_distance(waypt):
    """
    Computes the distance from current waypoint
    to ideal waypoint using DOFs, but might change to coord.
    ---
    input trajectory, output scalar feature
    """
    global ideal_config
    robot.SetDOFValues(waypt,manip.GetArmIndices())
    #ideal_waypt = get_ideal_config()
    ideal_waypt = ideal_config
    dist = np.linalg.norm(waypt-ideal_waypt)
    return dist

def cost_distance(waypt):
    """
    Computes the total distance between waypt and ideal waypt
    ---
    input trajectory, output scalar cost
    """

    feature = features_distance(waypt)
    #return feature*self.weights
    return feature

def features_distance_bet_deltas(waypt):
    """
    Computes the distance between delta EE
    and delta joint vectors 
    ---
    input trajectory, output scalar feature
    """
    global robot, manip, ideal_config
    #ideal_waypt = get_ideal_config()
    armids = list(manip.GetArmIndices())

    robot.SetDOFValues(waypt,armids)
    arm_transformations = np.array(robot.GetLinkTransformations())[armids]

    EEcurr = manip.GetEndEffectorTransform()[:3][:,3]
    robot.SetDOFValues(ideal_config,armids)
    ideal_arm_transformations = np.array(robot.GetLinkTransformations())[armids]
    EEdesired = manip.GetEndEffectorTransform()[:3][:,3]
    deltaEE = EEdesired-EEcurr

    dot_prods =[]
    for x in range(len(arm_transformations)):
        curr_pos = arm_transformations[x][:3][:,3]
        ideal_pos = ideal_arm_transformations[x][:3][:,3] 
        deltaJoint = ideal_pos-curr_pos
        dot_prods.append(np.dot(deltaEE,deltaJoint))
        
    return sum(dot_prods)

def cost_distance_bet_deltas(waypt):
    """
    Computes the total distance between waypt and ideal waypt
    ---
    input trajectory, output scalar cost
    """

    feature = features_distance_bet_deltas(waypt)
    return -feature


def cost_largest_change1(waypt):
    """
    Computes the total distance to table cost.
    ---
    input trajectory, output scalar cost
    """
    feature = features_largest_change(waypt,0)
    #return feature*self.weights
    return -feature

def cost_largest_change2(waypt):
    """
    Computes the total distance to table cost.
    ---
    input trajectory, output scalar cost
    """
    feature = features_largest_change(waypt,1)
    #return feature*self.weights
    return -feature

def cost_largest_change3(waypt):
    """
    Computes the total distance to table cost.
    ---
    input trajectory, output scalar cost
    """
    feature = features_largest_change(waypt,2)
    #return feature*self.weights
    return -feature

def cost_largest_change4(waypt):
    """
    Computes the total distance to table cost.
    ---
    input trajectory, output scalar cost
    """
    feature = features_largest_change(waypt,3)
    #return feature*self.weights
    return -feature

def cost_largest_change5(waypt):
    """
    Computes the total distance to table cost.
    ---
    input trajectory, output scalar cost
    """
    feature = features_largest_change(waypt,4)
    #return feature*self.weights
    return -feature

def cost_largest_change6(waypt):
    """
    Computes the total distance to table cost.
    ---
    input trajectory, output scalar cost
    """
    feature = features_largest_change(waypt,5)
    #return feature*self.weights
    return -feature

def cost_largest_change7(waypt):
    """
    Computes the total distance to table cost.
    ---
    input trajectory, output scalar cost
    """
    feature = features_largest_change(waypt,6)
    #return feature*self.weights
    return -feature