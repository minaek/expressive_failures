from __future__ import division
# from expressive_failures.util.util import rotation_z
import numpy as np, math
import json
# import openravepy
from openravepy import *
import trajoptpy
# from lfd.environment import sim_util
from lfd.util import util
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

#COST_FN_XSG = lambda x,s,g: cost_distance_bet_deltas(x, s, g, coeff=20)
COST_FN_XSG = lambda x,s,g: cost_projections(x, s, g, d=3, coeff=10)
#COST_FN_XSG = lambda x,s,g: cost_fun2(x, s, g, alpha=1, coeff=100)

def best_starting_config(): 
    """
    Optimizes for the best starting configuration.

    returns starting configuration, goal configuration, and trajectory
    """
    global sols,starting_configs,goal_config_stationary
    trajs = [] #list of lists
    costs = [] #list of lists
    print "len of sols: " + str(len(sols))
    for s in range(len(sols)):
        print s
        candidate_attempt_traj = trajopt(starting_configs[s],goal_config_stationary)
        trajs.append(candidate_attempt_traj) #append trajectory for a given ik solution
        waypt_costs = [] 
        for waypt in candidate_attempt_traj:  # TODO: Could consider only the last waypt's cost
            #waypt_costs.append(cost_projections(waypt,starting_configs[s],goal_config,coeff=1000)) #calculate cost for each waypoint
            #waypt_costs.append(cost_distance_bet_deltas(waypt,starting_configs[s],goal_config,coeff=1000)) #calculate cost for each waypoint
            waypt_costs.append(COST_FN_XSG(waypt, starting_configs[s], goal_config_stationary))
        costs.append(waypt_costs) #appending list of costs

    idx = np.argmin([sum(x) for x in costs]) #find the trajectory with the lowest cost sum
    print "COSTS: " + str(costs)
    print "idx: " + str(idx)
    print "smallest vals: " + str(costs[idx])
    traj = trajs[idx]
    starting_config = starting_configs[idx]
    return starting_config, goal_config_stationary, traj, trajs


def interpolate(start, goal, num_waypts):
    """
    Linear interpolation from start to goal in C-space.
    """
    init_waypts = np.zeros((num_waypts,7))
    for count in range(num_waypts):
        init_waypts[count,:] = start + count/(num_waypts - 1.0)*(goal - start)
    return init_waypts


def trajopt(starting_config,goal_config):
    #global n_steps, Final_pose, manip, ideal_config, ideal_config_vanilla
    n_steps = 15
    #ideal_config = [-1.0, -1, -1, -1.7, 1.7, 0, 0]

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
    robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices()], #set arm and base as active dofs
                    DOFAffine.X + DOFAffine.Y + DOFAffine.RotationAxis, [0,0,1])

    #filling in request dictionary
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "active",
            "start_fixed" : False
        },
        "costs" : [
        {
            "type" : "joint_vel",
            #"params": {"coeffs" : [gamma/(n_steps-1)]}
            "params": {"coeffs" : [1]}
        },            
        ],
        "constraints" : [],
        "init_info" : {
           # #"type": "given_traj",
           # #"data": init_traj.tolist()
           # #"type": "straight_line",
           # #"endpoint": globalvars.goal_config
           "type": "stationary",
        }   
    }

    #constraint:
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
    with openravepy.RobotStateSaver(robot):
      with util.suppress_stdout():
        prob = trajoptpy.ConstructProblem(s, robot.GetEnv()) # create object that stores optimization problem
        cost_fn = lambda x: COST_FN_XSG(x, starting_config, goal_config)
        #for t in range(n_steps):
        #    prob.AddCost(cost_fn, [(t,j) for j in range(7)], "table%i"%t)
        prob.AddCost(cost_fn, [(n_steps-1,j) for j in range(7)], "table%i"%(n_steps-1))

        print "optimizing prob to get result."
        result = trajoptpy.OptimizeProblem(prob) # do optimization
        print "done optimizing prob to get result."

    traj = result.GetTraj()
    dof_inds = sim_util.dof_inds_from_name(robot, manip_name)
    sim_util.unwrap_in_place(traj, dof_inds)
    
    return traj 

def visualize(traj):
    for _ in range(5):
        robot = env.GetRobots()[0]
        manip = robot.GetActiveManipulator()

        robot.SetDOFValues(traj[0],manip.GetArmIndices())
        time.sleep(.5)

        robot.SetDOFValues(traj[-1],manip.GetArmIndices())
        time.sleep(.5)

def executePathSim(waypts, reps=3, t=0.1):
    """
    Executes in the planned trajectory in simulation
    """
    global env,robot,manip
    
    #waypts = np.insert(waypts,0,starting_config,axis=0)
    #trajs = [waypts, np.flip(waypts,0)]
    for _ in range(reps):
        for w in waypts:
            robot.SetDOFValues(w,manip.GetArmIndices())
            time.sleep(t)

def executeBoth(waypts, starting_config, reps=3, t=0.1):
    global robot
    trans = robot.GetTransform()
    for _ in range(reps):
        reset = robot.GetTransform()
        reset[:3][:,3][:2] = [0,0]
        robot.SetTransform(reset)
        print "RESET!"
        robot.SetDOFValues(starting_config, manip.GetArmIndices())
        for w in waypts:
            trans[:3][:,3][:2] = w[-3:-1]
            robot.SetTransform(trans)
            robot.SetDOFValues(w[:7],manip.GetArmIndices())
            time.sleep(t)

def resetbase():
    reset = robot.GetTransform()
    reset[:3][:,3][:2] = [0,0]
    robot.SetTransform(reset)

def main():
    global curr_base,robot,manip,starting_config
    #s, g, traj = single_starting_config()
    s, g, traj, all_trajs = best_starting_config()
    executePathSim(traj, reps=1)


    # b,s,traj,base_costs, base_idx, starting_idx,total_sols = best_base_pos()
    # time.sleep(2)
    # for _ in range(3):
    #     interpolate_base(curr_base, num_points=15)
    #     robot.SetDOFValues(starting_config,manip.GetArmIndices())
    #     #executePathSim(reversed(traj), reps=1, t=0.006)
    #     time.sleep(.5)
    #     interpolate_base(b, num_points=20)
    #     robot.SetDOFValues(s, manip.GetArmIndices())
    #     executePathSim(traj, reps=1, t=0.1)
    #     time.sleep(.1)

    import IPython as ipy
    ipy.embed()

if __name__ == "__main__":
    main()
 
