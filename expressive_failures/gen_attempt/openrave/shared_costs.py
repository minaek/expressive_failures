from __future__ import division
import numpy as np, math

"""
Cost Function 1
"""

def cost_projections(shared, waypt, starting_config, goal_config, link_names, d=7, coeff=1, alpha=0.3):
    """
    The the second cost function we came up with that projects EL onto EE d times
    """
    feature = features_projections(shared, waypt, starting_config, goal_config, link_names, d, alpha)
    return feature*coeff

def features_projections(shared, waypt, starting_config, goal_config, link_names, d, alpha):
    """
    The the second cost function we came up with that projects EL onto EE d times
    """
    deltaEEs,deltaLs = get_deltas(shared, waypt, link_names, starting_config, goal_config)
    norm_deltaEEs = [np.linalg.norm(deltaEE) for deltaEE in deltaEEs]
    norm_deltaLs = [np.linalg.norm(deltaL) for deltaL in deltaLs]
    projections = []

    for l in range(len(link_names)):
        if norm_deltaLs[l] < 1e-10:
            proj = 0
        else:
            init_projection = np.dot(deltaEEs[l],deltaLs[l])/norm_deltaEEs[l]
            cos_theta = np.dot(deltaEEs[l],deltaLs[l])/(norm_deltaEEs[l] * norm_deltaLs[l])
            proj = (init_projection*(cos_theta**(d-1)))
        projections.append(proj)

    return alpha - sum(projections)

def get_deltas(shared, waypt, link_names, starting_config, goal_config):
    """
    Computes deltaEE and deltaEL

    """
    robot = shared.robot
    manip = shared.manip
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()

    assert len(waypt) == len(armids) or len(waypt) == len(armids)+2
    include_base = len(waypt) > len(armids)

    deltaEEs=[]
    deltaLs = []
    waypt_arm = waypt[:len(armids)]
    if include_base:
        trans = robot.GetTransform()
        orig_trans = np.array(trans)
        waypt_basexy = waypt[len(armids):]
    orig_config = robot.GetDOFValues(armids)

    for link_name in link_names:
        link_idx = links.index(robot.GetLink(link_name))
        n_coord = 3
        if link_name == "torso_lift_motor_screw_link":
            # Only consider xy, not xyz
            n_coord = 2

        #current (q)
        # PREVIOUS VERSION: Include "and n_coord == 2" in line below
        if include_base:
        #if include_base and n_coord == 2:
            trans[:2,3] = waypt_basexy
            robot.SetTransform(trans)
        robot.SetDOFValues(waypt_arm,armids) # set dof values to current config
        linkstrans = robot.GetLinkTransformations()
        Lcurr = linkstrans[link_idx][:n_coord,3] #get xyz of specific link

        #set robot to starting base position of [0,0]
        # PREVIOUS VERSION: Include "and n_coord == 2" in line below
        if include_base:
        #if include_base and n_coord == 2:
            trans[:2,3] = [0,0]
            robot.SetTransform(trans)

        #ideal (q_d)
        robot.SetDOFValues(goal_config,armids) #set dof valuue to ideal config
        EEdesired = manip.GetEndEffectorTransform()[:n_coord,3] #get EE qd    
        
        #start (q_s)
        robot.SetDOFValues(starting_config,armids) #set dof valuue to starting config
        linkstrans_s = robot.GetLinkTransformations()
        Lstart = linkstrans_s[link_idx][:n_coord,3]
        EEstart = manip.GetEndEffectorTransform()[:n_coord,3]

        deltaEEs.append(EEdesired - EEstart)
        deltaLs.append(Lcurr - Lstart)

    # Reset robot to original position
    if include_base:
        robot.SetTransform(orig_trans)
    robot.SetDOFValues(orig_config, armids)

    return deltaEEs, deltaLs


"""
Cost Function 2
"""
def cost_match_positions(shared, waypt, starting_config, goal_config, link_names, coeff=1, alpha=0.3):
    """
    """
    feature = features_l2_distance(shared, waypt, starting_config, goal_config, link_names, alpha) #elbow

    return feature*coeff

def features_l2_distance(shared, waypt, starting_config, goal_config, link_names, alpha):
    robot = shared.robot
    manip = shared.manip
    armids = list(manip.GetArmIndices()) #get arm indices

    assert len(waypt) == len(armids) or len(waypt) == len(armids)+2
    include_base = len(waypt) > len(armids)
    waypt_arm = waypt[:len(armids)]
    if include_base:
        trans = robot.GetTransform()
        orig_trans = np.array(trans)
        waypt_basexy = waypt[len(armids):]
    orig_config = robot.GetDOFValues(armids)

    links = robot.GetLinks()
    l2norm_sqs = []

    for link_name in link_names:
        if link_name == "torso_lift_motor_screw_link":  # Ignore base
            continue
        link_idx = links.index(robot.GetLink(link_name))

        #current EL coord
        if include_base:
            trans[:2,3] = waypt_basexy
            robot.SetTransform(trans)
        robot.SetDOFValues(waypt_arm, armids)
        linkstrans = robot.GetLinkTransformations()
        ELq = linkstrans[link_idx][:3,3]

        #desired EL coord
        if include_base:
            trans[:2,3] = [0,0]
            robot.SetTransform(trans)
        robot.SetDOFValues(goal_config, armids)
        linkstrans_d = robot.GetLinkTransformations()
        ELd = linkstrans_d[link_idx][:3,3]
        l2norm_sq = np.linalg.norm(ELq-ELd)**2 #l2 norm, squared
        l2norm_sqs.append(l2norm_sq)

    # Reset robot to original position
    if include_base:
        robot.SetTransform(orig_trans)
    robot.SetDOFValues(orig_config, armids)

    return alpha + np.sum(l2norm_sqs)

"""
Cost Function 3
"""
def cost_alignment(waypt, starting_config, goal_config, coeff=1):
    """
    """
    feature = features_alignment(waypt, starting_config, goal_config) 

    return feature*coeff

def features_alignment(waypt, starting_config, goal_config):
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
	EEd = manip.GetEndEffectorTransform()[:3][:,3]
	linkstrans_d = robot.GetLinkTransformations()
	Ld = linkstrans[link_idx][:3][:,3]

	deltaQ = EEq - Lq
	deltaD = EEd - Ld

	return np.dot(deltaQ, deltaD)

"""
Cost Function 4
"""
def cost_similarity_in_cspace(waypt, starting_config, goal_config, coeff=1):
    """
    """
    feature = features_similarity_in_cspace(waypt, starting_config, goal_config) #elbow

    return feature*coeff

def features_similarity_in_cspace(waypt, starting_config, goal_config):
	global robot, manip
	q = waypt
	qd = goal_config
	delta = q-qd
	return 0.1-np.dot(q,qd)


"""
Base
"""
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

    # Set robot to starting base position of [0,0]
    trans[:,3][:2] = [0,0]
    robot.SetTransform(trans)

    linkstrans = robot.GetLinkTransformations()
    Bstart = linkstrans[link_idx][:,3][:2] #get xy of specific link

    #ideal
    robot.SetDOFValues(goal_config,armids) #set dof value to ideal config
    EEdesired = manip.GetEndEffectorTransform()[:3][:,3][:2] #get EE qd    
 
    #start
    robot.SetDOFValues(starting_config,armids) #set dof valuue to starting config
    EEstart = manip.GetEndEffectorTransform()[:3][:,3][:2]

    Bdes = (EEdesired-EEstart)
    Bdelta = (Bcurr-Bstart)

    norm_ideal_base = np.linalg.norm(Bdes)
    norm_curr_base = np.linalg.norm(Bdelta)
    if norm_curr_base < 1e-10:
        proj = 0
    else:
        init_projection = np.dot(Bdes, Bdelta)/norm_ideal_base
        cos_theta = np.dot(Bdes, Bdelta)/(norm_curr_base*norm_ideal_base)
        proj = (init_projection*(cos_theta**(d-1)))

    scalar = .2-proj
    #print proj
    #print scalar 
    return scalar*coeff #this works better than the projection
