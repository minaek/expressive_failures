from __future__ import division
import globalvars
from globalvars import *
import numpy as np, math
from math import *
import time
import scipy.spatial.distance

"""All cost functions"""

def get_deltas(waypt, link_name,starting_config,goal_config):
    """
    Computes deltaEE and deltaEL

    """
    global robot, manip
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink(link_name))

    #current
    robot.SetDOFValues(waypt,armids) # set dof values to current config
    EEcurr = manip.GetEndEffectorTransform()[:3][:,3] #get EE q
    linkstrans = robot.GetLinkTransformations()
    Lcurr = linkstrans[link_idx][:3][:,3] #get xyz of specific link

    #ideal
    robot.SetDOFValues(goal_config,armids) #set dof valuue to ideal config
    EEdesired = manip.GetEndEffectorTransform()[:3][:,3] #get EE qd    
    linkstrans_d = robot.GetLinkTransformations()
    Ldesired = linkstrans_d[link_idx][:3][:,3]
    
    #start
    robot.SetDOFValues(starting_config,armids) #set dof valuue to starting config
    linkstrans_s = robot.GetLinkTransformations()
    Lstart = linkstrans_s[link_idx][:3][:,3]
    EEstart = manip.GetEndEffectorTransform()[:3][:,3]

    deltaEE = (EEdesired-EEstart)
    deltaL = (Lcurr-Lstart)
    return deltaEE, deltaL

def get_rot_deltas(waypt, link_name, starting_config, goal_config):
    global robot, manip
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink(link_name))

    #current
    robot.SetDOFValues(waypt,armids) # set dof values to current config
    linkstrans = robot.GetLinkTransformations()
    Lcurr_rot =  linkstrans[link_idx][:3][:,:3]

    #desired
    robot.SetDOFValues(goal_config, armids)
    EEdes_rot = manip.GetEndEffectorTransform()[:3][:,:3]

    #start
    robot.SetDOFValues(starting_config,armids)
    EEstart_rot = manip.GetEndEffectorTransform()[:3][:,:3]
    linkstrans_s = robot.GetLinkTransformations()
    Lstart_rot = linkstrans_s[link_idx][:3][:,:3]

    deltaEE_rot = np.dot(EEdes_rot,np.linalg.inv(EEstart_rot))
    deltaL_rot = np.dot(Lcurr_rot,np.linalg.inv(Lstart_rot))

    return deltaEE_rot, deltaL_rot

def features_fun2(waypt, starting_config, goal_config, alpha):
    """
    The similarity + alpha*magnitude cost function
    """
    link_name = 'r_elbow_flex_link'
    deltaEE,deltaL = get_deltas(waypt, link_name, starting_config, goal_config)

    norm_deltaEE = np.linalg.norm(deltaEE)
    norm_deltaL = np.linalg.norm(deltaL)
    #similarity = 1- scipy.spatial.distance.cosine(deltaEE,deltaL)
    similarity = np.dot(deltaEE,deltaL)/(norm_deltaEE * norm_deltaL)
    theta = np.arccos(similarity)
    magnitude = alpha*norm_deltaL
    if norm_deltaL < 1e-10:
        cosdist = 0.1
        dist = 0.1
        thetadist = 0.1
    else: 
        cosdist = (1 - similarity) / 20  # Range: [0, 0.1]
        print "cosdist: " + str(cosdist)
        thetadist = theta / 30
        print "thetadist: " + str(thetadist)
        dist = 0.1 - magnitude
        print "dist: " + str(dist)
    # print "similarity: " + str(magnitude)

    # print ".1-similarity: " + str(.1-(magnitude))

    #return 0.1-distance
    return 0.0001*thetadist + dist

def cost_fun2(waypt, starting_config, goal_config, alpha=1, coeff=1):
    """
    The similarity + alpha*magnitude cost function
    """
    feature = features_fun2(waypt,starting_config,goal_config,alpha)
    print "feature: " + str(feature)
    print "feature*coeff: " + str(feature*coeff) + "\n"
    return feature*coeff

def features_projections_helper(deltaEE, deltaL, d):
    norm_deltaEE = np.linalg.norm(deltaEE)
    norm_deltaL = np.linalg.norm(deltaL)

    if norm_deltaL < 1e-10:
        proj = 0
    else:
        init_projection = np.dot(deltaEE,deltaL)/norm_deltaEE
        cos_theta = np.dot(deltaEE,deltaL)/(norm_deltaEE * norm_deltaL)
        proj = (init_projection*(cos_theta**(d-1)))
    return proj 

def features_projections(waypt, starting_config, goal_config, d):
    """
    The the second cost function we came up with that projects EL onto EE d times
    """
    link_name = 'r_elbow_flex_link'
    deltaEE,deltaL = get_deltas(waypt, link_name, starting_config, goal_config)
    # deltaEE_x, deltaEE_y, deltaEE_z, deltaL_x, deltaL_y, deltaL_z = \
    #     get_rot_deltas(waypt, link_name, starting_config, goal_config)

    # deltaEE_rot, deltaL_rot = get_rot_deltas(waypt, link_name, starting_config, goal_config)

    # proj_x = features_projections_helper(deltaEE_rot[:3][:,0], deltaL_rot[:3][:,0],d)
    # proj_y = features_projections_helper(deltaEE_rot[:3][:,1], deltaL_rot[:3][:,1],d)
    # proj_z = features_projections_helper(deltaEE_rot[:3][:,2], deltaL_rot[:3][:,2],d)
    # # proj_p = features_projections_helper(deltaEE, deltaL,d)


    # proj_x = np.dot(deltaEE_rot[:3][:,0], deltaL_rot[:3][:,0])
    # proj_y = np.dot(deltaEE_rot[:3][:,1], deltaL_rot[:3][:,1])
    # proj_z = np.dot(deltaEE_rot[:3][:,2], deltaL_rot[:3][:,2])

    norm_deltaEE = np.linalg.norm(deltaEE)
    norm_deltaL = np.linalg.norm(deltaL)

    if norm_deltaL < 1e-10:
        proj = 0
    else:
        init_projection = np.dot(deltaEE,deltaL)/norm_deltaEE
        cos_theta = np.dot(deltaEE,deltaL)/(norm_deltaEE * norm_deltaL)
        proj = (init_projection*(cos_theta**(d-1)))

    return 0.1 - proj

def cost_projections(waypt, starting_config, goal_config, d=7, coeff=1):
    """
    The the second cost function we came up with that projects EL onto EE d times
    """
    feature = features_projections(waypt, starting_config, goal_config, d)
    return feature*coeff


def feature_base(waypt):
    global base_positions,robot
    for base in base_positions:
        transform = robot.GetTransform()#reset base to original position
        transform[:3][:,3] = base
        robot.SetTransform(transform) 
        

def cost_base(waypt, coeff=1):
    feature = features_base(waypt)
    return feature*coeff

def features_distance_bet_deltas(waypt,starting_config,goal_config):
    """
    Computes the distance between delta EE
    and delta joint vectors 
    ---
    input trajectory, output scalar feature
    """
    link_name = 'r_elbow_flex_link'
    deltaEE, deltaL = get_deltas(waypt, link_name,starting_config,goal_config)
    norm_deltaEE = np.linalg.norm(deltaEE)
    #print ""
    #print(norm_deltaEE)
    norm_deltaL = np.linalg.norm(deltaL)
    #print(norm_deltaL)
    alpha = .01
    cos_theta = np.dot(deltaEE,deltaL)/(norm_deltaEE * norm_deltaL)
    #print(cos_theta)
    #distance functions
    dotprod = np.dot(deltaEE,deltaL)
    proj = dotprod / norm_deltaEE
    #d = np.linalg.norm(deltaEE-deltaL)
    #cos_dist = scipy.spatial.distance.cosine(deltaEE,deltaL)
    # print "DOT PRODUCT: " + str(dotprod)

    # print "DOT PRODUCT: " + str(1-dotprod)

    return .1 - proj

def cost_distance_bet_deltas(waypt,starting_config,goal_config,coeff=1,):
    """
    Computes the total distance between waypt and ideal waypt
    ---
    input trajectory, output scalar cost
    """

    feature = features_distance_bet_deltas(waypt,starting_config,goal_config)
    return feature*coeff


