from __future__ import division
import numpy as np, math

"""
Cost Function 1: imitates desired movement of end effector
"""
def cost_phi_ee(shared, waypt, starting_config, goal_config, link_names, dist_f, coeff=1, alpha=0.3):
    phi_ee, phi_js = get_xyzs(shared, waypt, link_names, starting_config, goal_config)
    phi_ee_qs, phi_ee_qd = phi_ee

    total_dist = 0
    for phi_j_qs, phi_j_qd, phi_j_q in phi_js:
        phi_ee_delta = phi_ee_qd - phi_ee_qs
        if len(phi_j_q) == 2:
            phi_ee_delta = phi_ee_delta[:2]
        total_dist += dist_f(phi_j_q - phi_j_qs, phi_ee_delta)
    return coeff*(alpha + total_dist)

"""
Cost Function 2: imitates desired movement of body parts
"""
def cost_phi_bodypts(shared, waypt, starting_config, goal_config, link_names, dist_f, coeff=1, alpha=0.3):
    if 'torso_lift_motor_screw_link' in link_names:
        link_names.remove('torso_lift_motor_screw_link')
    _, phi_js = get_xyzs(shared, waypt, link_names, starting_config, goal_config)

    total_dist = 0
    for phi_j_qs, phi_j_qd, phi_j_q in phi_js:
        total_dist += dist_f(phi_j_q - phi_j_qs, phi_j_qd - phi_j_qs)
    return coeff*(alpha + total_dist)

"""
Cost Function 3: imitates desired change in configuration
"""
def cost_config(shared, waypt, starting_config, goal_config, link_names, dist_f, coeff=1, alpha=0.3):
    # Note: only care about configuration of arm, not about location of base
    armids = list(shared.manip.GetArmIndices())
    waypt_arm = waypt[:len(armids)]
    dist = dist_f(waypt_arm - starting_config, goal_config - starting_config)
    return coeff*(alpha + dist)

"""
Cost Function Utils
"""
def get_xyzs(shared, waypt, link_names, starting_config, goal_config):
    robot = shared.robot
    manip = shared.manip
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()

    assert len(waypt) == len(armids) or len(waypt) == len(armids)+2
    include_base = len(waypt) > len(armids)

    phi_ee = None
    phi_js = []
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
        if include_base:  #if include_base and n_coord == 2:
            trans[:2,3] = waypt_basexy
            robot.SetTransform(trans)
        robot.SetDOFValues(waypt_arm,armids) # set dof values to current config
        linkstrans = robot.GetLinkTransformations()
        phi_j_q = linkstrans[link_idx][:n_coord,3] #get xyz of specific link

        #set robot to starting base position of [0,0]
        # PREVIOUS VERSION: Include "and n_coord == 2" in line below
        if include_base:  #if include_base and n_coord == 2:
            trans[:2,3] = [0,0]
            robot.SetTransform(trans)

        #ideal (q_d)
        robot.SetDOFValues(goal_config,armids) #set dof valuue to ideal config
        linkstrans_d = robot.GetLinkTransformations()
        phi_j_qd = linkstrans_d[link_idx][:n_coord,3] #get xyz of specific link
        phi_ee_qd = manip.GetEndEffectorTransform()[:n_coord,3] #get EE qd    
        
        #start (q_s)
        robot.SetDOFValues(starting_config,armids) #set dof valuue to starting config
        linkstrans_s = robot.GetLinkTransformations()
        phi_j_qs = linkstrans_s[link_idx][:n_coord,3]
        phi_ee_qs = manip.GetEndEffectorTransform()[:n_coord,3]

        if phi_ee is None:
            phi_ee = (phi_ee_qs, phi_ee_qd)
        phi_js.append((phi_j_qs, phi_j_qd, phi_j_q))

    # Reset robot to original position
    if include_base:
        robot.SetTransform(orig_trans)
    robot.SetDOFValues(orig_config, armids)

    return phi_ee, phi_js

