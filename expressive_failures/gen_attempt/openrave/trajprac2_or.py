from __future__ import division
# from expressive_failures.util.util import rotation_z
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
import lfd.environment


def attempt_traj():
    """ 
    Computes the actual attempt trajectory
    ---
    input: ee constraint coordinate position in global frame, ideal traj output: trajectory
    """    
    global robot,manip,starting_config
    robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices()], \
                    DOFAffine.X + DOFAffine.Y + DOFAffine.RotationAxis, [0,0,1])
    robot.SetDOFValues(starting_config, manip.GetArmIndices())
    xyz_target = list(manip.GetEndEffectorTransform()[:3][:,3])
    return plan_follow_trajs(robot,manip_name, xyz_target)

def interpolate(start, goal, num_waypts):
    """
    Linear interpolation from start to goal in C-space.
    """
    global manip
    # lfd.environment.sim_util.unwrap_in_place(start[np.newaxis,:], list(manip.GetArmIndices()))
    # lfd.environment.sim_util.unwrap_in_place(goal[np.newaxis,:], list(manip.GetArmIndices()))

    init_waypts = np.zeros((num_waypts,7))
    for count in range(num_waypts):
        init_waypts[count,:] = start + count/(num_waypts - 1.0)*(goal - start)

    return init_waypts


def plan_follow_trajs(robot, manip_name, xyz_target):
    global n_steps, starting_config, manip, goal_config
 
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

    link_idx = links.index(robot.GetLink('r_gripper_tool_frame'))
    linkstrans = robot.GetLinkTransformations()
    xyz_target = list(linkstrans[link_idx][:3][:,3]) #get xyz of specific link
    quat_target = list(Quaternion(matrix=linkstrans[link_idx]))
    #computing initial trajectory 
    #goal_config = iksolver(ideal_config)
    init_traj = interpolate(starting_config, goal_config, n_steps)
    init_traj2 = [[-0.3362314 ,  0.64878883, -1.14855121, -1.20951067, -2.,
        -0.10000004, -0.4       ],
       [-0.3329129 ,  0.65634106, -1.13246233, -1.21026465, -1.97265453,
        -0.10947545, -0.43998464],
       [-0.32754992,  0.66784973, -1.10754933, -1.21147136, -1.94060464,
        -0.12424174, -0.49141749],
       [-0.31855059,  0.68552363, -1.06827563, -1.21346357, -1.90721458,
        -0.14758598, -0.55489722],
       [-0.30318169,  0.71196863, -1.00687747, -1.21677086, -1.87955661,
        -0.18390186, -0.62845002],
       [-0.27810863,  0.74777834, -0.91751313, -1.22190863, -1.86617333,
        -0.23586539, -0.70615278],
       [-0.24058753,  0.78988714, -0.79990115, -1.22899755, -1.87001658,
        -0.3021847 , -0.78260856],
       [-0.18864887,  0.83334106, -0.65640946, -1.23761741, -1.88789799,
        -0.37975909, -0.85647948],
       [-0.36109314,  0.26212034, -1.84618206, -1.20368534, -1.88      ,
        -0.38571587, -3.68911704],
       [-0.36449141,  0.27749054, -1.81979484, -1.20286485, -1.88      ,
        -0.36702095, -3.70711847]] 
    #filling in request dictionary
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : manip_name,
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            #"params": {"coeffs" : [gamma/(n_steps-1)]}
            "params": {"coeffs" : [1]}
        },            
        ],
        "constraints" : [
        # {
        # "type" : "joint_vel",
        # "name" : "joint_vel",
        # "params" : {
        #     "max_displacement" : 0.05,
        #     "first_step" : 0,
        #     "last_step": n_steps-1
        #     }
        # }

        ],

        "init_info" : {
            "type": "given_traj",
            "data": init_traj.tolist()
            # "type": "given_traj",
            # "data": init_traj.tolist()
            # "type": "straight_line",
            # "endpoint": goal_config
        }   
    }

    for t in range(n_steps):
        pose_constraint = \
        {                                                                             
            "type" : "pose", 
            "params" : {
            "xyz" : xyz_target1, 
            "wxyz" : quat_target1, 
            "link": "r_gripper_palm_link",
            "timestep" : t,
            "rot_coeffs" : [2,2,2],
            "pos_coeffs" : [5,5,5]
            }
        }
        gripper_constraint = \
        {                                                                             
            "type" : "pose", 
            "params" : {
            "xyz" : xyz_target, 
            "wxyz" : quat_target, 
            "link": "r_gripper_tool_frame",
            "timestep" : t,
            "rot_coeffs" : [2,2,2],
            "pos_coeffs" : [5,5,5]
            }
        }

        request["constraints"].append(pose_constraint)
        #request["constraints"].append(gripper_constraint)

    #robot.SetDOFValues(starting_config, manip.GetArmIndices())
    s = json.dumps(request)
    # with openravepy.RobotStateSaver(robot):
    #   with util.suppress_stdout():
    prob = trajoptpy.ConstructProblem(s, robot.GetEnv()) # create object that stores optimization problem
    for t in range(n_steps):
        prob.AddCost(cost_distance_bet_deltas, [(t,j) for j in range(7)], "table%i"%t)
        #prob.AddCost(cost_distance, [(t,j) for j in range(7)], "table%i"%t)

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

    for _ in range(3):
        robot.SetDOFValues(traj[0],manip.GetArmIndices())
        time.sleep(1)

        robot.SetDOFValues(traj[-1],manip.GetArmIndices())
        time.sleep(1)
    robot.SetDOFValues(traj[0],manip.GetArmIndices())
    

def executePathSim(waypts):
    """
    Executes in the planned trajectory in simulation
    """
    global env,robot,manip,starting_config
    for _ in range(3):
        for w in waypts:
            robot.SetDOFValues(w,manip.GetArmIndices())
            time.sleep(.4)
        #robot.SetDOFValues(starting_config,manip.GetArmIndices())
        # robot.SetDOFValues(waypts[9], manip.GetArmIndices())
        # robot.SetDOFValues(waypts[8], manip.GetArmIndices())

        # robot.SetDOFValues(waypts[7], manip.GetArmIndices())
        # robot.SetDOFValues(waypts[5], manip.GetArmIndices())
        # robot.SetDOFValues(waypts[4], manip.GetArmIndices())
        # robot.SetDOFValues(waypts[3], manip.GetArmIndices())

        # robot.SetDOFValues(waypts[2], manip.GetArmIndices())
        # time.sleep(.2)

        #time.sleep(.1)
        # for w in reversed(range(len(waypts))):
        #     robot.SetDOFValues(waypts[w],manip.GetArmIndices())
        #     time.sleep(.2)