from __future__ import division
# from expressive_failures.util.util import rotation_z
import numpy as np, math
import json
# import openravepy
from openravepy import *
import trajoptpy
# from lfd.environment import sim_util
# from lfd.util import util
# from lfd.rapprentice import math_utils as mu, resampling, retiming
# from lfd.transfer import settings
import time
# import trajoptpy.kin_utils as ku
# import argparse
from math import *
import cost
from cost import *
from pyquaternion import Quaternion
import globalvars
from lfd.environment import sim_util


def attempt_traj():
    """ 
    Computes the actual attempt trajectory
    ---
    input: ee constraint coordinate position in global frame, ideal traj output: trajectory
    """    
    robot = globalvars.or_sim.env.GetRobots()[0]
    manip = robot.GetActiveManipulator()

    robot.SetDOFValues(globalvars.starting_config, manip.GetArmIndices())
    xyz_target = list(manip.GetEndEffectorTransform()[:3][:,3])
    manip_name = "rightarm"
    return plan_follow_trajs(robot,manip_name,manip, xyz_target, globalvars.starting_config)

def interpolate(start, goal, num_waypts):
    """
    Linear interpolation from start to goal in C-space.
    """
    init_waypts = np.zeros((num_waypts,7))
    for count in range(num_waypts):
        init_waypts[count,:] = start + count/(num_waypts - 1.0)*(goal - start)
    return init_waypts


def plan_follow_trajs(robot, manip_name, manip, xyz_target,starting_config):
    #global n_steps, Final_pose, manip, ideal_config, ideal_config_vanilla
    n_steps = 100
    ideal_config = [-1.0, -1, -1, -1.7, 1.7, 0, 0]

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
        #{                                                                             
        #    "type" : "pose", 
        #    "params" : {
        #    "xyz" : xyz_target1, 
        #    "wxyz" : quat_target1, 
        #    "link": "r_gripper_palm_link",
        #    "timestep" : n_steps-1,
        #    "rot_coeffs" : [2,2,2],
        #    "pos_coeffs" : [5,5,5]
        #    }

        #    # "type" : "joint", # joint-space target
        #    # "params" : {"vals" : Final_pose.tolist() } # length of vals = # dofs of manip
        #},

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

    for t in range(n_steps):
        pose_constraint = {"type": "pose",
                        "params" : {
                        "xyz" : xyz_target1, 
                        "wxyz" : quat_target1, 
                        "link": "r_gripper_palm_link",
                        "timestep" : t,
                        "rot_coeffs" : [2,2,2],
                        "pos_coeffs" : [5,5,5]
                        }}
        request["constraints"].append(pose_constraint)
    #robot.SetDOFValues(starting_config, manip.GetArmIndices())
    s = json.dumps(request)
    # with openravepy.RobotStateSaver(robot):
    #   with util.suppress_stdout():
    prob = trajoptpy.ConstructProblem(s, robot.GetEnv()) # create object that stores optimization problem
    cost_fn = lambda x: cost_distance_bet_deltas(x, coeff=1)
    for t in range(n_steps):
        prob.AddCost(cost_fn, [(t,j) for j in range(7)], "table%i"%t)

    print "optimizing prob to get result."
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    print "done optimizing prob to get result."

    traj = result.GetTraj()
    dof_inds = sim_util.dof_inds_from_name(robot, manip_name)
    sim_util.unwrap_in_place(traj, dof_inds)
    
    print " "
    print "Resulting Traj: "
    print traj
    return traj 


def visualize(traj):
    for _ in range(5):
        robot = globalvars.or_sim.env.GetRobots()[0]
        manip = robot.GetActiveManipulator()


        robot.SetDOFValues(traj[0],manip.GetArmIndices())
        time.sleep(.5)

        robot.SetDOFValues(traj[-1],manip.GetArmIndices())
        time.sleep(.5)

def executePathSim(waypts):
    """
    Executes in the planned trajectory in simulation
    """
    global env,robot,manip
    
    #waypts = np.insert(waypts,0,starting_config,axis=0)
    #trajs = [waypts, np.flip(waypts,0)]

    for w in waypts:
        print (w)
        robot.SetDOFValues(w,manip.GetArmIndices())
        time.sleep(.2)
