from __future__ import division
import globalvars
from globalvars import *
import numpy as np, math
from math import *
import time
import scipy.spatial.distance

"""
Cost Function 1
"""

def cost_projections(waypt, starting_config, goal_config, d=7, coeff=1):
    """
    The the second cost function we came up with that projects EL onto EE d times
    """
    feature = features_projections(waypt, starting_config, goal_config, d) #elbow+shoulder
    #feature = features_projection(waypt, starting_config, goal_config, d) #elbow

    return feature*coeff

def features_projections(waypt, starting_config, goal_config, d):
    """
    The the second cost function we came up with that projects EL onto EE d times
    """
    link_name = 'r_elbow_flex_link'
    link_names = ['r_elbow_flex_link', 'r_shoulder_lift_link']
    deltaEEs,deltaLs = get_deltas(waypt, link_names, starting_config, goal_config)
    norm_deltaEEs = []
    norm_deltaLs = []
    projections = []
    for l in range(len(link_names)):
        norm_deltaEE = np.linalg.norm(deltaEEs[l])
        norm_deltaEEs.append(norm_deltaEE)
        norm_deltaL = np.linalg.norm(deltaLs[l])
        norm_deltaLs.append(norm_deltaL)

    for l in range(len(link_names)):
        if norm_deltaLs[l] < 1e-10:
            proj = 0
        else:
            init_projection = np.dot(deltaEEs[l],deltaLs[l])/norm_deltaEEs[l]
            cos_theta = np.dot(deltaEEs[l],deltaLs[l])/(norm_deltaEEs[l] * norm_deltaLs[l])
            proj = (init_projection*(cos_theta**(d-1)))
        projections.append(proj)

    return 0.1 - sum(projections)

def get_deltas(waypt, link_names, starting_config, goal_config):
    """
    Computes deltaEE and deltaEL

    """
    global robot, manip
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    deltaEEs=[]
    deltaLs = []
    for link_name in link_names:
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
        deltaEEs.append(deltaEE)
        deltaL = (Lcurr-Lstart)
        deltaLs.append(deltaL)

    return deltaEEs, deltaLs


"""
Cost Function 2
"""
def l1_distance(waypt, starting_config, goal_config):
    global robot, manip
    armids = list(manip.GetArmIndices()) #get arm indices
    link_name = 'r_elbow_flex_link'
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink(link_name))

    #current EL coord
    robot.SetDOFValues(waypt, armids)
    linkstrans = robot.GetLinkTransformations()
    ELq = linkstrans[link_idx][:3][:,3]

    #desired EL coord
    robot.SetDOFValues(goal_config, armids)
    linkstrans_d = robot.GetLinkTransformations()
    ELd = linkstrans_d[link_idx][:3][:,3]
    l1norm = np.linalg.norm((ELq-ELd), ord=1)

    return 0.1 - l1norm

"""
Cost Function 3
"""
def alignment(waypt, starting_config, goal_config):
	global robot, manip
    armids = list(manip.GetArmIndices()) #get arm indices
    link_name = 'r_shoulder_lift_link'
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink(link_name))

    #current EE, L
    robot.SetDOFValues(waypt, armids)
    EEq = manip.GetEndEffectorTransform()[:3][:,3] #get EE q
    linkstrans = robot.GetLinkTransformations()
    Lq = linkstrans[link_idx][:3][:,3] #get xyz of specific link

    #desired L
    robot.SetDOFValues(goal_config, armids)
    linkstrans_d = robot.GetLinkTransformations()
    Ld = linkstrans[link_idx][:3][:,3]

    deltaQ = EEq - Lq
    deltaD = EEq - Ld

    return np.dot(deltaQ, deltaD)
