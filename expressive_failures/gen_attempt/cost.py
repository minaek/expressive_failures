import globalvars
from globalvars import *
import numpy as np, math
from math import *
import time

"""All cost functions"""



####### (1) Minimize distance between delta vectors #######
def features_distance_bet_deltas(waypt):
    """
    Computes the distance between delta EE
    and delta joint vectors 
    ---
    input trajectory, output scalar feature
    """
    global robot, manip, ideal_config, ideal_config_vanilla, Final_pose
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink('r_elbow_flex_link'))
    link_idxs = [links.index(robot.GetLink('r_elbow_flex_link')),links.index(robot.GetLink('torso_lift_link'))]
    curr=[]
    des =[]
    dotprods = []

    robot.SetDOFValues(waypt,armids) # set dof values to current config
    EEcurr = manip.GetEndEffectorTransform()[:3][:,3] #get EE q
    linkstrans = robot.GetLinkTransformations()
    Lcurr = linkstrans[link_idx][:3][:,3] #get xyz of specific link
    # for i in range(len(link_idxs)):
    #     curr.append(linkstrans[link_idxs[i]][:3][:,3])


    robot.SetDOFValues(ideal_config,armids) #set dof valuue to ideal config
    EEdesired = manip.GetEndEffectorTransform()[:3][:,3] #get EE qd    
    linkstrans_d = robot.GetLinkTransformations()
    Ldesired = linkstrans_d[link_idx][:3][:,3]
    # for i in range(len(link_idxs)):
    #     des.append(linkstrans_d[link_idxs[i]][:3][:,3])

    deltaEE = np.abs(EEdesired-EEcurr)
    deltaL = np.abs(Ldesired-Lcurr)
    # for i in range(len(link_idxs)):
    #     deltaLs = des[i]-curr[i]
    #     dotprods.append(np.dot(deltaEE,deltaLs))


    dotprod = np.dot(deltaEE,deltaL)
    #d = np.linalg.norm(deltaEE-deltaL)

    return dotprod
    # return sum(dotprods)

    # dot_prods =[]
    # for x in range(len(arm_transformations)):
    #     curr_pos = arm_transformations[x][:3][:,3]
    #     ideal_pos = ideal_arm_transformations[x][:3][:,3] 
    #     deltaJoint = ideal_pos-curr_pos
    #     dot_prods.append(np.dot(deltaEE,deltaJoint))
    # print "DOT PROD LIST: " + str(dot_prods), print sum(dot_prods)
    # return sum(dot_prods)

def cost_distance_bet_deltas(waypt):
    """
    Computes the total distance between waypt and ideal waypt
    ---
    input trajectory, output scalar cost
    """

    feature = features_distance_bet_deltas(waypt)
    return feature


####### (2) Minimize distance between desired and actual joint config #######
def features_distance(waypt):
    """
    Computes the distance from current waypoint
    to ideal waypoint using DOFs, but might change to coord.
    ---
    input trajectory, output scalar feature
    """
    global ideal_config, ideal_config_vanilla, Final_pose
    robot.SetDOFValues(waypt,manip.GetArmIndices())
    #ideal_waypt = get_ideal_config()
    ideal_waypt = ideal_config
    dist = np.linalg.norm(ideal_waypt-waypt)
    #dist = np.sum(np.abs(waypt-ideal_waypt))
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


####### (3) Maximize similarity between delta config and goal vectors #######
def features_ee_to_joint_dist(waypt):
    global robot, manip, ideal_config,ideal_config_vanilla,Final_pose
    curr=[]
    des=[]
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    names = ['r_elbow_flex_link','r_forearm_link','r_upper_arm_link','r_shoulder_lift_link']
    sum_dot_prod=0


    robot.SetDOFValues(waypt,armids) # set dof values to current config
    EEcurr = manip.GetEndEffectorTransform()[:3][:,3] #get EE q
    for name in names:
        link_idx = links.index(robot.GetLink(name))
        linkstrans = robot.GetLinkTransformations()
        Lcurr = linkstrans[link_idx][:3][:,3] #get xyz of specific link
        curr.append(EEcurr-Lcurr)

    robot.SetDOFValues(ideal_config,armids) #set dof valuue to ideal config
    EEdes = manip.GetEndEffectorTransform()[:3][:,3] #get EE qd  
    for name in names:
        link_idx = links.index(robot.GetLink(name))
        linkstrans = robot.GetLinkTransformations()
        Ldes = linkstrans[link_idx][:3][:,3] #get xyz of specific link
        des.append(EEdes-Ldes)

    for i in range(len(curr)):
        sum_dot_prod = sum_dot_prod + np.dot(curr[i],des[i])
    return sum_dot_prod


def cost_ee_to_joint_dist(waypt):
    feature = features_ee_to_joint_dist(waypt)
    return -feature


####### (4) Maximize similarity in config space #######
def features_config(waypt):
    global ideal_config,robot,ideal_config_vanilla,Final_pose
    robot.SetDOFValues(waypt,manip.GetArmIndices())
    ideal_waypt = ideal_config
    diff = waypt-ideal_waypt
    return np.dot(diff,diff)

def cost_config(waypt):
    feature = features_config(waypt)
    return feature



#######################################################
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