from __future__ import division
import globalvars
from globalvars import *
import numpy as np, math
from math import *
import time
import scipy.spatial.distance

"""All cost functions"""

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

    if (np.array(norm_deltaLs) < 1e-10).all():
        proj = 0
    else:
        for l in range(len(link_names)):
            init_projection = np.dot(deltaEEs[l],deltaLs[l])/norm_deltaEEs[l]
            cos_theta = np.dot(deltaEEs[l],deltaLs[l])/(norm_deltaEEs[l] * norm_deltaLs[l])
            proj = (init_projection*(cos_theta**(d-1)))
            projections.append(proj)

    return 0.1 - sum(projections)


def base(waypt, starting_config, goal_config, d=3,coeff=1):
    armids = list(manip.GetArmIndices()) #get arm indices
    trans = robot.GetTransform()
    curr_xy = waypt[-2:]
    trans[:3][:,3][:2] = curr_xy
    robot.SetTransform(trans)
    links = robot.GetLinks()
    #link_idx = links.index(robot.GetLink('base_laser_link'))
    link_idx = links.index(robot.GetLink("torso_lift_motor_screw_link"))
    #link_idx = links.index(robot.GetLink("r_shoulder_lift_link"))

    linkstrans = robot.GetLinkTransformations()
    Bcurr = linkstrans[link_idx][:3][:,3][:2] #get xy of specific link

    #ideal
    robot.SetDOFValues(goal_config,armids) #set dof value to ideal config
    EEdesired = manip.GetEndEffectorTransform()[:3][:,3][:2] #get EE qd    
 
    #start
    robot.SetDOFValues(starting_config,armids) #set dof valuue to starting config
    EEstart = manip.GetEndEffectorTransform()[:3][:,3][:2]

    Bdes = (EEdesired-EEstart)

    norm_ideal_base = np.linalg.norm(Bdes)
    norm_curr_base = np.linalg.norm(Bcurr)
    if norm_curr_base < 1e-10:
        proj = 0
    else:
        init_projection = np.dot(Bdes, Bcurr)/norm_ideal_base
        cos_theta = np.dot(Bdes, Bcurr)/(norm_curr_base*norm_ideal_base)
        proj = (init_projection*(cos_theta**(d-1)))

    print proj
    print scalar 
    scalar = .2-proj
    return scalar*coeff #this works better than the projection 


def get_delta(waypt, link_name,starting_config,goal_config):
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

def features_projection(waypt, starting_config, goal_config, d):
    """
    The the second cost function we came up with that projects EL onto EE d times
    """
    link_name = 'r_elbow_flex_link'
    deltaEE,deltaL = get_delta(waypt, link_name, starting_config, goal_config)
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
    feature = features_projections(waypt, starting_config, goal_config, d) #elbow+shoulder
    #feature = features_projection(waypt, starting_config, goal_config, d) #elbow

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


