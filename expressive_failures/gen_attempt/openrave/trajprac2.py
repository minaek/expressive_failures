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
COST_FN_XSG = lambda x,s,g: cost_projections(x, s, g, d=3, coeff=20)
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
        candidate_attempt_traj = attempt_traj(starting_configs[s],goal_config_stationary)
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


def interpolate_base(new_xyz, num_points=7):
    transform = robot.GetTransform()
    curr_xyz = transform[:3][:,3]
    delta = (new_xyz-curr_xyz)/num_points

    for i in range(num_points):
        curr_xyz+=delta
        transform[:3][:,3] = curr_xyz 
        robot.SetTransform(transform)   

def move_robot(new_xyz):
    transform = robot.GetTransform()
    transform[:3][:,3] = new_xyz
    robot.SetTransform(transform)

def best_base_pos():
    global base_positions,robot,manip, total_sols, goal_config, curr_base
    #Total sols contains base positions
    #base positions contains a list of ik solutions
    #evaluate each ik solution and find the shit that has the lowest cost overall
    #that will be your base position and config you move into when you move to that base, then execute traj
    #but your actual starting config should be one from the current base position 

    #TOTAL SOLS:
        #[B1[IK[7 dof], IK[7 dof]],IK[]], B2[]]
    base_costs = [] #list of list 
    print "calculating costs for all trajs.."
    for b in range(len(total_sols)):
        print "base number: " + str(b)
        move_robot(base_positions[b])
        ik_costs = [] #list 
        for ik in range(len(total_sols[b])):
            candidate_attempt_traj = attempt_traj(total_sols[b][ik], goal_config)
            waypt_costs=0
            for waypt in candidate_attempt_traj:
                waypt_costs+=(COST_FN_XSG(waypt, total_sols[b][ik], goal_config))
            #print "WAYPOT COSTS: " + str(waypt_costs)
            ik_costs.append(waypt_costs)
            #print len(ik_costs)
        base_costs.append(ik_costs)

    base_idx = None
    starting_idx = None 
    curr_min = np.infty
    #base[iks[4,6], ik[waypt_costs[]]]

    print "finding smallest cost traj.."
    #print base_costs
    for b in range(len(base_costs)):
        if (len(base_costs[b])!=0):
            candidate_min = min(base_costs[b])
            if curr_min>candidate_min:
                curr_min = candidate_min
                base_idx = b
                starting_idx = base_costs[b].index(curr_min)

    print "starting idx: " + str(starting_idx)
    #print total_sols[base_idx]
    best_base_position = base_positions[base_idx]
    best_starting_position = total_sols[base_idx][starting_idx]
    print "solving for best trajectory.."
    best_traj = attempt_traj(best_starting_position, goal_config)

    print "Done!"
    return best_base_position, best_starting_position, best_traj, base_costs, base_idx, starting_idx,total_sols

def single_starting_config():
    """
    Computes trajectory with a single starting configuration that we specify.

    returns starting configuration, goal configuration, and trajectory
    """
    global robot,manip
    
    #starting_config = manip.GetArmDOFValues()
    starting_config = joint_start_gazebo #SPECIFY SPECIFIC STARTING CONFIG HERE

    goal_transform = manip.GetEndEffectorTransform()
    goal_transform[:3][:,3][2] += 0.3
    goal_config,_ = iksolver(goal_transform)
    traj = attempt_traj(starting_config,goal_config)

    return starting_config,goal_config,traj

def attempt_traj(starting_config,goal_config):
    """ 
    Computes the actual attempt trajectory
    ---
    input: ee constraint coordinate position in global frame, ideal traj output: trajectory
    """    
    robot = env.GetRobots()[0]
    manip = robot.GetActiveManipulator()

    robot.SetDOFValues(starting_config, manip.GetArmIndices())
    xyz_target = list(manip.GetEndEffectorTransform()[:3][:,3])
    manip_name = "rightarm"
    return plan_follow_trajs(robot,manip_name,manip, xyz_target, starting_config,goal_config)

def interpolate(start, goal, num_waypts):
    """
    Linear interpolation from start to goal in C-space.
    """
    init_waypts = np.zeros((num_waypts,7))
    for count in range(num_waypts):
        init_waypts[count,:] = start + count/(num_waypts - 1.0)*(goal - start)
    return init_waypts


def plan_follow_trajs(robot, manip_name, manip, xyz_target,starting_config,goal_config):
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

    #computing initial trajectory 
    #goal_config = iksolver(ideal_config)
    #
    #init_traj = interpolate(starting_config, goal_config, n_steps)
    #init_traj = interpolate(starting_config, starting_config, n_steps)

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
        "constraints" : [],
        "init_info" : {
           # #"type": "given_traj",
           # #"data": init_traj.tolist()
           # #"type": "straight_line",
           # #"endpoint": globalvars.goal_config
           "type": "stationary",
        }   
    }
    robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices()],DOFAffine.X + DOFAffine.Y + DOFAffine.RotationAxis, [0,0,1])

    # #init traj: 
    # # BEGIN random_init
    # # randomly select joint values with uniform distribution over joint limits
    # lower,upper = robot.GetActiveDOFLimits()
    # lower = np.clip(lower, -np.pi, np.pi) # continuous joints have huge ranges, so we clip them to [-pi, pi]
    # upper = np.clip(upper, -np.pi, np.pi) # to avoid poor numerical conditioning
    # rands = np.random.rand(len(lower))
    # dofvals_init = lower*rands + upper*(1-rands)
    # # we'll treat the base pose specially, choosing a random angle and then setting a reasonable
    # # position based on this angle
    # angle_init = np.random.rand() * 2*np.pi
    # x_init = xyz_target1[0] - .5*np.cos(angle_init)
    # y_init = xyz_target1[1] - .5*np.sin(angle_init)    
    # dofvals_init[-3:] = [x_init, y_init, angle_init]
    # # END random_init

    # request["init_info"]["type"] = "given_traj"
    # request["init_info"]["data"] = [dofvals_init.tolist()]

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
    
    # print " "
    # print "Resulting Traj: "
    # print traj

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
 
