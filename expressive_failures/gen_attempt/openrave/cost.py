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

def features_fun2(waypt, starting_config, goal_config):
    """
    The similarity + alpha*magnitude cost function
    """
    link_name = 'r_elbow_flex_link'
    deltaEE,deltaL = get_deltas(waypt, link_name, starting_config, goal_config)

    norm_deltaEE = np.linalg.norm(deltaEE)
    norm_deltaL = np.linalg.norm(deltaL)
    alpha = 1
    similarity = 1- scipy.spatial.distance.cosine(deltaEE,deltaL)

    return 1-(similarity+(alpha*norm_deltaL))

def cost_fun2(waypt, starting_config, goal_config, coeff=1):
    """
    The similarity + alpha*magnitude cost function
    """
    feature = features_fun2(waypt,starting_config,goal_config)
    return feature*coeff

def features_projections(waypt, starting_config, goal_config, d):
    """
    The the second cost function we came up with that projects EL onto EE d times
    """
    link_name = 'r_elbow_flex_link'
    deltaEE,deltaL = get_deltas(waypt, link_name, starting_config, goal_config)

    norm_deltaEE = np.linalg.norm(deltaEE)
    norm_deltaL = np.linalg.norm(deltaL)

    init_projection = np.dot(deltaEE,deltaL)/norm_deltaEE
    cos_theta = np.dot(deltaEE,deltaL)/(norm_deltaEE * norm_deltaL)

    return 1-(init_projection*(cos_theta**(d-1)))

def cost_projections(waypt, starting_config, goal_config, d=7, coeff=1):
    """
    The the second cost function we came up with that projects EL onto EE d times
    """
    feature = features_projections(waypt, starting_config, goal_config, d)
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
    #d = np.linalg.norm(deltaEE-deltaL)
    #cos_dist = scipy.spatial.distance.cosine(deltaEE,deltaL)
    # print "DOT PRODUCT: " + str(dotprod)

    # print "DOT PRODUCT: " + str(1-dotprod)

    return -dotprod

def cost_distance_bet_deltas(waypt,starting_config,goal_config,coeff=1,):
    """
    Computes the total distance between waypt and ideal waypt
    ---
    input trajectory, output scalar cost
    """

    feature = features_distance_bet_deltas(waypt,starting_config,goal_config)
    return feature*coeff


