from __future__ import division
from expressive_failures.util.util import rotation_z
import numpy as np, math
import json
import openravepy
from openravepy import *
import trajoptpy
from lfd.environment import sim_util
from lfd.util import util
from lfd.rapprentice import math_utils as mu, resampling, retiming
from lfd.transfer import settings
import time
import trajoptpy.kin_utils as ku
import argparse
from math import *
import util_attempt,cost,globalvars
from util_attempt import *
from cost import *
from globalvars import *
from pyquaternion import Quaternion


def attempt_traj():
    """ 
    Computes the actual attempt trajectory
    ---
    input: ee constraint coordinate position in global frame, ideal traj output: trajectory
    """    
    global robot,manip,starting_config

    robot.SetDOFValues(starting_config, manip.GetArmIndices())
    xyz_target = list(manip.GetEndEffectorTransform()[:3][:,3])
    return plan_follow_trajs(robot,manip_name, xyz_target)

def interpolate(start, goal, num_waypts):
    """
    Linear interpolation from start to goal in C-space.
    """
    init_waypts = np.zeros((num_waypts,7))
    for count in range(num_waypts):
        init_waypts[count,:] = start + count/(num_waypts - 1.0)*(goal - start)
    return init_waypts


def plan_follow_trajs(robot, manip_name, xyz_target):
    global n_steps, starting_config, Final_pose, manip, ideal_config, ideal_config_vanilla
 
    #quat_target = [1,0,0,0] # wxyz
    robot.SetDOFValues(starting_config, manip.GetArmIndices())
    # t = manip.GetEndEffectorTransform()
    # quat_target = [x for x in (list(Quaternion(matrix=t)))]
    #quat_target = list(transform2quat(manip.GetEndEffectorTransform()))
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink('r_gripper_palm_link'))
    linkstrans = robot.GetLinkTransformations()
    xyz_target1 = list(linkstrans[link_idx][:3][:,3]) #get xyz of specific link
    quat_target1 = list(Quaternion(matrix=linkstrans[link_idx]))

    #computing initial trajectory 
    #goal_config = iksolver(ideal_config)
    init_traj = interpolate(starting_config, ideal_config, n_steps)

    #filling in request dictionary
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : manip_name,
            #"start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            #"params": {"coeffs" : [gamma/(n_steps-1)]}
            "params": {"coeffs" : [1]}
        },            
        ],
        "constraints" : [
        {                                                                             
            "type" : "pose", 
            "params" : {
            "xyz" : xyz_target1, 
            "wxyz" : quat_target1, 
            "link": "r_gripper_palm_link",
            "timestep" : n_steps-1,
            "rot_coeffs" : [2,2,2],
            "pos_coeffs" : [5,5,5]
            }

            # "type" : "joint", # joint-space target
            # "params" : {"vals" : Final_pose.tolist() } # length of vals = # dofs of manip
        },

        # {
        #     "type" : "pose", 
        #     "params" : {
        #     "xyz" : xyz_wrist_target, 
        #     "wxyz" : quat_target, 
        #     "link": "r_gripper_r_link",
        #     "timestep" : n_steps-1,
        #     "rot_coeffs" : [0,0,0],
        #     "pos_coeffs" : [1,1,1]
        #     }


        # }

        ],

        "init_info" : {
            "type": "given_traj",
            "data": init_traj.tolist()
            #"type": "straight_line",
            #"enpoint": init_joint_target.tolist()
        }   
    }
    #robot.SetDOFValues(starting_config, manip.GetArmIndices())
    s = json.dumps(request)
    # with openravepy.RobotStateSaver(robot):
    #   with util.suppress_stdout():
    prob = trajoptpy.ConstructProblem(s, robot.GetEnv()) # create object that stores optimization problem
    for t in range(n_steps):
        prob.AddCost(cost_distance_bet_deltas, [(t,j) for j in range(7)], "table%i"%t)

    print "optimizing prob to get result."
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    print "done optimizing prob to get result."

    traj = result.GetTraj()
    
    print " "
    print "Resulting Traj: "
    print traj
    return traj 


def visualize(traj):
    global env,robot,manip,starting_config

    for _ in range(5):
        robot.SetDOFValues(traj[0],manip.GetArmIndices())
        time.sleep(.5)

        robot.SetDOFValues(traj[-1],manip.GetArmIndices())
        time.sleep(.5)

def executePathSim(waypts):
    """
    Executes in the planned trajectory in simulation
    """
    global env,robot,manip,starting_config
    
    #waypts = np.insert(waypts,0,starting_config,axis=0)
    #trajs = [waypts, np.flip(waypts,0)]

    for w in waypts:
        print (w)
        robot.SetDOFValues(w,manip.GetArmIndices())
        time.sleep(.2)

# def get_ideal_config():
#     """
#     Getter for ideal configuration
#     ---
#     output configuration
#     """
#     global LIFTING_GOAL
#     print "Getting ideal trajectory."
#     traj = ideal_traj(LIFTING_GOAL)
#     print "Done getting ideal trajectory."
#     return traj[-1]
