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
import util,cost,globalvars
from util import *
from cost import *
from globalvars import *

def attempt_traj():
    """ 
    Computes the actual attempt trajectory
    ---
    input: ee constraint coordinate position in global frame, ideal traj output: trajectory

    """    
    global robot,manip,starting_config

    robot.SetDOFValues(starting_config, manip.GetArmIndices())
    xyz_target = list(manip.GetEndEffectorTransform()[:3][:,3])

    return plan_follow_trajs(robot,manip_name, xyz_target, "A")

def ideal_traj(xyz_target):
    """ 
    Computes ideal trajectory to goal target position 
    ---
    input: goal coordinate position in global frame, output: trajectory

    """
    global ee_link_name,manip_name,nsteps, ee_link_z

    traj =  plan_follow_trajs(robot,manip_name,xyz_target, "I")

    robot.SetDOFValues(traj[-1], manip.GetArmIndices())
    ee_transform = manip.GetEndEffectorTransform()

    print " "
    print "current pos: " + str(ee_transform[:3][:,3])
    print "goal pos: " + str(ee_link_z) 
    print "starting pos: " + str(starting_transform[:3][:,3])

    return traj

def interpolate(start, goal, num_waypts):
    """
    Linear interpolation from start to goal in C-space.
    """
    init_waypts = np.zeros((num_waypts,7))
    for count in range(num_waypts):
        init_waypts[count,:] = start + count/(num_waypts - 1.0)*(goal - start)
    return init_waypts


def plan_follow_trajs(robot, manip_name, xyz_target, IorA):
    print "ENTERED PLAN FOLLORW TRAJS"
    global n_steps, starting_config, Final_pose, manip, LIFTING_GOAL
 
    #quat_target = [1,0,0,0] # wxyz
    quat_target = list(transform2quat(manip.GetEndEffectorTransform()))

    hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

    if IorA == "A":
        print "computing init traj."
        init_traj = interpolate(starting_config, Final_pose, n_steps)
        print "done computing init traj."

    elif IorA == "I":
        init_traj = interpolate(starting_config, Final_pose, n_steps)
    else:
        raise

    print "init_traj:"
    print init_traj
 
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : manip_name,
            #"start_fixed" : start_fixed
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
            "params" : {"xyz" : xyz_target, 
            "wxyz" : quat_target, 
            "link": "r_gripper_tool_frame",
            "timestep" : n_steps-1
            }

            # "type" : "joint", # joint-space target
            # "params" : {"vals" : Final_pose.tolist() } # length of vals = # dofs of manip
        }],

        "init_info" : {
            "type": "given_traj",
            "data": init_traj.tolist()
            #"type": "straight_line",
            #"enpoint": init_joint_target.tolist()
        }   
    }
    robot.SetDOFValues(starting_config, manip.GetArmIndices())
    y,p,r = get_euler(manip.GetEndEffectorTransform())
    request["constraints"][0]["params"]["rot_coeffs"] = [0,0,0]
    #request["constraints"][0]["params"]["rot_coeffs"] = [y,p,r]

    
    print "computing s"
    s = json.dumps(request)
    print "done computing s"
    #with openravepy.RobotStateSaver(robot):
   # with util.suppress_stdout():
    print "S: " + str(s)
    print "computing prob"
    prob = trajoptpy.ConstructProblem(s, robot.GetEnv()) # create object that stores optimization problem
    print "done computing prob."
    if IorA == "I":
        for t in range(n_steps): 
            prob.AddCost(cost_largest_change1, [(t,j) for j in range(7)], "table%i"%t)
            prob.AddCost(cost_largest_change2, [(t,j) for j in range(7)], "table%i"%t)
            prob.AddCost(cost_largest_change3, [(t,j) for j in range(7)], "table%i"%t)
            prob.AddCost(cost_largest_change4, [(t,j) for j in range(7)], "table%i"%t)
            prob.AddCost(cost_largest_change5, [(t,j) for j in range(7)], "table%i"%t)
            prob.AddCost(cost_largest_change6, [(t,j) for j in range(7)], "table%i"%t)
            prob.AddCost(cost_largest_change7, [(t,j) for j in range(7)], "table%i"%t)

            #prob.AddCost(cost_quat, [(t,j) for j in range(7)], "table%i"%t)
    elif IorA == "A":
        print"Adding cost."
        for t in range(n_steps):
            #Minimize distance between delta EE and delta EL vecs
            prob.AddCost(cost_distance_bet_deltas, [(t,j) for j in range(7)], "table%i"%t)

            #minimize distance between current config and ideal config 
            #prob.AddCost(cost_distance, [(t,j) for j in range(7)], "table%i"%t)
        print "Done adding cost."
    else:
        raise

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

def get_ideal_config():
    """
    Getter for ideal configuration
    ---
    output configuration
    """
    global LIFTING_GOAL
    print "Getting ideal trajectory."
    traj = ideal_traj(LIFTING_GOAL)
    print "Done getting ideal trajectory."
    return traj[-1]