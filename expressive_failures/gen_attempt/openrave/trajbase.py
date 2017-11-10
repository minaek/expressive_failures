from __future__ import division

# import argparse
# parser = argparse.ArgumentParser()
# parser.add_argument("--interactive", action="store_true")
# args = parser.parse_args()


import trajoptpy
import numpy as np
import json
from trajoptpy.check_traj import traj_is_safe
import numpy as np, math
import json
from openravepy import *
from lfd.util import util
import time
from math import *
from collection_of_costs import *
from pyquaternion import Quaternion
import globalvars
from lfd.environment import sim_util


#LINK_NAMES = ['r_elbow_flex_link', 'r_shoulder_lift_link']
LINK_NAMES = ['r_elbow_flex_link', 'r_shoulder_lift_link', 'torso_lift_motor_screw_link']
#COST_FN_XSG = lambda x,s,g: cost_projections(x, s, g, LINK_NAMES, d=3, coeff=20)
COST_FN_XSG = lambda x,s,g: cost_projections(x, s, g, LINK_NAMES, d=3, coeff=20, alpha=0.3)

#COST_FN_XSG = lambda x,s,g: cost_distance_bet_deltas(x, s, g, coeff=20)
#COST_FN_BASE = lambda x, s, g: base(x,s,g,d=3, coeff=20)

N_STEPS=20
FAST = 2.7
MEDIUM = 0.9
SLOW = 0.3

BASE = True #flag. If True, we optimize for base as well 
EXCLUDE_GRIPPER_COLLISIONS_FOR_ATTEMPT = True

def best_starting_config(): 
    """
    Optimizes for the best starting configuration.

    returns starting configuration, goal configuration, and trajectory
    """
    global starting_configs,goal_config_stationary,Tgoal
    trajs = [] #list of lists
    costs = [] #list of lists
    results = [] #contains result from solving TrajOpt problems
    print "# possible starting configs: " + str(len(starting_configs))
    for i,start_config in enumerate(starting_configs):
        print i
        candidate_attempt_traj, result = position_base_request(start_config,goal_config_stationary)
        trajs.append(candidate_attempt_traj) #append trajectory for a given ik solution for q_s
        results.append(result)
        # total cost from COST_FN_XSG and COST_FN_BASE
        cost = np.sum([val for x,val in result.GetCosts() if x=="f"])

        # check to see whether pose constraints were violated at any timestep
        pose_violation = np.max([val for name,val in result.GetConstraints() if "pose" in name])
        if pose_violation > 1e-3:
            cost += 1000
        
        # check to see whether collision costs are zero
        coll_cost = np.sum([y for x,y in result.GetCosts() if x.startswith("collision")])
        if coll_cost > 1e-3:
            cost += 10000
        costs.append(cost) #appending list of costs

    idx = np.argmin(costs) #find the trajectory with the lowest cost sum
    traj = trajs[idx]
    best_start_config = starting_configs[idx]
    return best_start_config, goal_config_stationary, traj, trajs, result, costs, idx, results

# def calculate_EE_movement(traj):
#     deviation=0

#     EE_constrained_pos = Tgoal[:3][:,3]

#     for waypt in traj:
#         robot.SetDOFValues(waypt, manip.GetArmIndices())
#         EE_pos = manip.GetEndEffectorTransform()[:3][:,3]
#         deviation+=np.sum(np.abs(EE_constrained_pos-EE_pos))

#     return deviation/N_STEPS

def calculate_EL_BASE_movement(traj, axis, direction, init_position=[0,0]):
    #axis = x:0, y:1, z:2
    #direction = -1 or +1
    #[[1,2,3],[3,4,5]]
    elbow_flag = False #True if elbow is moving in correct direction
    base_flag = False #True if base is moving in correct direction 
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    elbow_link_idx = links.index(robot.GetLink('r_elbow_flex_link'))
    base_link_idx = links.index(robot.GetLink("torso_lift_motor_screw_link"))
    EL_positions = []
    BASE_positions = []

    # for waypt in traj:
    #     #appending elbow and base xyz positions for each waypoint
    #     robot.SetDOFValues(waypt, armids)
    #     linkstrans = robot.GetLinkTransformations()
    #     EL_pos = linkstrans[elbow_link_idx][:3][:,3]
    #     EL_positions.append(EL_pos)
    #     BASE_pos = linkstrans[base_link_idx][:3][:,3]
    #     BASE_positions.append(BASE_pos)

    waypt = traj[-1]
    robot.SetDOFValues(waypt, armids)
    linkstrans = robot.GetLinkTransformations()
    EL_pos = linkstrans[elbow_link_idx][:3][:,3]
    EL_positions.append(EL_pos)
    BASE_pos = linkstrans[base_link_idx][:3][:,3]
    BASE_positions.append(BASE_pos)


    if np.sign(BASE_pos[axis]-init_position[axis])==np.sign(direction):
        base_flag =True
    if np.sign(EL_pos[axis]-Tgoal[:3][:,3][axis])-np.sign(direction):
        elbow_flag = True 

    #EL_values_along_axis = np.array(EL_positions)[:,axis]
    #BASE_values_along_axis = np.array(BASE_positions)[:,axis]
    # if (np.diff(EL_values_along_axis) == direction).all():
    #     elbow_flag = True 
    # if ((np.diff(BASE_values_along_axis)==direction).all() or axis==2):
    #     base_flag = True 
    return elbow_flag, base_flag 


# def calculate_waypt_cost(waypt,starting_config):
#     waypt_cost = COST_FN_XSG(waypt, starting_configs[s], goal_config_stationary) + \
#      COST_FN_BASE(waypt, starting_configs[s],goal_config_stationary)

# waypt_cost = lambda waypt: calculate_waypt_cost(waypt, starting_config)

def search_over_costs(sol):
    """
    input: a single ik solution 
    output: a best cost and cost coefficient for the ik solution 
    """
    global COST_FN_XSG, COST_FN_BASE,goal_config_stationary, axis, direction
    best_cost = np.infty
    best_coeff = None
    best_traj = None
    for c in range(1,21):
        print "cost: " + str(c)
        COST_FN_XSG = lambda x,s,g: cost_projections(x, s, g, d=3, coeff=c) #I HOPE THIS IS UPDATING
        COST_FN_BASE = lambda x, s, g: base(x,s,g,d=3, coeff=c)
        candidate_traj,candidate_result = position_base_request(sol, goal_config_stationary)
        elbow_flag, base_flag = calculate_EL_BASE_movement(candidate_traj, axis, direction)
        if (elbow_flag is True and base_flag is True):
            #waypt_cost = lambda waypt: calculate_waypt_cost(waypt, starting_config)
            #np.apply_along_axis(waypt_cost, 1, candidate_attempt_traj) #returns an array of costs for the candidate attempt traj
            arm_cost, base_cost = candidate_result.GetCosts()[1:]
            curr_cost = arm_cost + base_cost
            if curr_cost < best_cost:
                best_cost = curr_cost 
                best_coeff = c 
                best_traj = candidate_traj
    print ""
    return best_cost, best_coeff, best_traj

def optimize_starting_and_cost():
    global sols,starting_configs,goal_config_stationary

    print "LEN OF SOLS: " + str(len(sols))
    #vfun = np.vectorize(search_over_costs)
    #costs = vfun(sols)
    costs = np.apply_along_axis(search_over_costs, 1, sols) #array of [best cost, best coeff] for each sol
    
    print "COSTS SHAPE"
    print np.shape(costs)
    idx = np.argmin(costs[:,0])
    best_cost, best_coeff, best_traj = costs[idx]
    best_starting_config = sols[idx]
    if best_cost==np.infty and best_coeff==None:
        print "There are no suitable cost coefficients and starting configs."
    return best_starting_config, goal_config_stationary, best_traj, best_cost, best_coeff

# def optimize_starting_and_cost():
#     #1. end effector moves less then ep amount
#     #2. elbow moves in required direction 
#     #3. base moves in required direction 
#     #find the smallest cost 
#     global sols,starting_configs,goal_config_stationary,Tgoal, COST_FN_XSG, COST_FN_BASE, axis, direction
#     trajs = [] #list of lists
#     best_costs = [] #(cost coefficient,cost) of best cost value for each starting config
#     epsilon = 1
#     print "len of sols: " + str(len(sols))
#     for s in range(len(sols)):
#         print s
#         costs_c = [] #list of lists
#     print best_costs
#     if best_costs==[]:
#         print "There are no valid solutions for this problem."
#         return 0
#     else:
#         idx = np.argmin(np.array(best_costs)[:,1])
#         bestest_cost_coeff = best_costs[idx][0]
#         bestest_cost =  best_costs[idx][1]
#         starting_config = sols[idx]
#         COST_FN_XSG = lambda x,s,g: cost_projections(x, s, g, d=3, coeff=bestest_cost_coeff) #I HOPE THIS IS UPDATING
#         COST_FN_BASE = lambda x, s, g: base(x,s,g,d=3, coeff=bestest_cost_coeff)
#         traj = position_base_request(starting_config, goal_config_stationary)
#         return starting_config, goal_config_stationary, traj, bestest_cost_coeff


def position_base_request(starting_config, goal_config, init_position=[0,0]):
    # init_position - initial position of robot's base
    global robot
    #seems like nsteps needs to be defined as 1 when the base is the only active DOF initialized
    n_steps = N_STEPS
    T = robot.GetTransform()
    T[:,3][:2] = init_position
    robot.SetTransform(T)
    armids = list(manip.GetArmIndices()) #get arm indices
    robot.SetDOFValues(starting_config, armids)
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink('r_gripper_palm_link'))
    linkstrans = robot.GetLinkTransformations()
    xyz_target = list(linkstrans[link_idx][:3][:,3]) #get xyz of specific link
    quat_target = list(Quaternion(matrix=linkstrans[link_idx])) #get quat of specific link
    if BASE:
        robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices()], DOFAffine.X + DOFAffine.Y)

    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "start_fixed" : True  # DOF values at first timestep are fixed based on current robot state
        },

        "costs" : [
        {
            "type" : "joint_vel",
            "params" : {"coeffs" : [1]}
        },
        {
            "type" : "collision",
            "params" : {
                "coeffs" : [1], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
              "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
                }   
        },
        ],
        "constraints" : [],
        "init_info" : {}
    }

    ##Initialize trajectory as stationary##:
    request["init_info"]["type"] = "stationary"
    request["basic_info"]["manip"] = "active" if BASE else "rightarm"

    for i in range(n_steps):
        request["constraints"].append({
            "type" : "pose",
            "name" : "pose"+str(i),
            "params" : {
                "pos_coeffs" : [6,6,6], #[6,6,6],
                "rot_coeffs" : [2,2,2], #[2,2,2],
                "xyz" : list(xyz_target),
                "wxyz" : list(quat_target),
                "link" : "r_gripper_palm_link",
                "timestep": i,
            }})

    s = json.dumps(request)
    prob = trajoptpy.ConstructProblem(s, env)

    cost_fn = lambda x: COST_FN_XSG(x, starting_config, goal_config_stationary)
    #cost_fn2  = lambda x: COST_FN_BASE(x, starting_config, goal_config_stationary)
    #for n in range(n_steps):
    #    prob.AddCost(cost_fn2, [(n,j) for j in range(7)], "base%i"%(n))
    with openravepy.RobotStateSaver(robot):
      with util.suppress_stdout():
        n_dof = 7
        if BASE:
            n_dof = 9
        prob.AddCost(cost_fn, [(n_steps-1,j) for j in range(n_dof)], "express%i"%(n_steps-1))
        #if BASE:
        #    prob.AddCost(cost_fn2, [(n_steps-1,j) for j in range(7,9)], "base%i"%(n_steps-1))
        result = trajoptpy.OptimizeProblem(prob)
    traj = result.GetTraj()
    dof_inds = sim_util.dof_inds_from_name(robot, manip_name)
    sim_util.unwrap_in_place(traj, dof_inds)
    return traj, result

def check_result(result, robot):
    print "checking trajectory for safety and constraint satisfaction..."
    success = True    
    if not traj_is_safe(result.GetTraj(), robot):
        success = False
        print "trajectory has a collision!"
    abstol = 1e-3
    for (name, val) in result.GetConstraints():
        if (val > abstol):
            success = False
            print "constraint %s wasn't satisfied (%.2e > %.2e)"%(name, val, abstol)
    return success

def visualize(traj):
    for _ in range(5):
        robot = env.GetRobots()[0]
        manip = robot.GetActiveManipulator()

        robot.SetDOFValues(traj[0],manip.GetArmIndices())
        time.sleep(.5)

        robot.SetDOFValues(traj[-1],manip.GetArmIndices())
        time.sleep(.5)

def executeArm(waypts, starting_config, attempt_speed=1.0):
    # attempt_speed, reset_speed - in units of change in configuration space per second

    global robot
    trans = robot.GetTransform()

    # Set robot to starting configuration
    robot.SetDOFValues(starting_config, manip.GetArmIndices())

    # Compute average distance between waypoints
    avg_norm = 0
    for idx in range(len(waypts)-1):
        avg_norm += np.linalg.norm(waypts[idx+1] - waypts[idx])
    avg_norm /= len(waypts) - 1
    print "avg_norm:", avg_norm

    t_attempt = avg_norm / attempt_speed

    # Attempt
    for w in waypts:
        robot.SetDOFValues(w[:7],manip.GetArmIndices())
        time.sleep(t_attempt)

def executeBase(waypts, reps = 3, t=0.1):
    global robot
    trans = robot.GetTransform()
    for _ in range(reps):
        for w in waypts:
            trans[:3][:,3][:2] = w[-2:]
            print w[-2:]
            robot.SetTransform(trans)
            time.sleep(t)

def trim(waypts):
    """
    Deletes redundant waypoints 
    """
    traj = waypts
    diff,sumd = difference_in_traj(traj)
    sumd = np.array(sumd)
    while ((sumd <= 1e-3).any()):
        for s in range(len(sumd)):
            if sumd[s] <=1e-3:
                traj = np.delete(traj, (s+1), axis=0)
                break 
        _,sumd = difference_in_traj(traj) 
        sumd = np.array(sumd)
    return traj




def executeBothTimed(waypts, starting_config, reps=3, \
                     attempt_speed=1.0, reset_speed=0.2, both=True, pause_btwn=0.05,
                     premotion=False):
    # attempt_speed, reset_speed - in units of change in configuration space per second
    # if premotion is True, this is executing heuristic baseline

    global robot
    trans = robot.GetTransform()

    # Set robot to starting configuration
    Tstart = robot.GetTransform()
    Tstart[:3][:,3][:2] = [0,0]
    robot.SetTransform(Tstart)
    robot.SetDOFValues(starting_config, manip.GetArmIndices())

    # Compute average distance between waypoints
    avg_norm = 0
    for idx in range(len(waypts)-1):
        avg_norm += np.linalg.norm(waypts[idx+1] - waypts[idx])
    avg_norm /= len(waypts) - 1
    print "avg_norm:", avg_norm

    t_attempt = avg_norm / attempt_speed
    t_reset = avg_norm / reset_speed

    if waypts.shape[1] == 7:
        both=False

    if both==True:
        if premotion:
            gripper_before()
            for w in waypts[::-1][1:]:
                trans[:3][:,3][:2] = w[-2:]
                robot.SetTransform(trans)
                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_reset)

        for i in range(reps):
            # Attempt
            for w in waypts:
                trans[:3][:,3][:2] = w[-2:]
                robot.SetTransform(trans)
                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_attempt)
            if premotion:
                gripper_after()

            time.sleep(pause_btwn)

            if premotion and i == reps-1:
                break
            if premotion:
                gripper_before()
            # Reset
            for w in waypts[::-1][1:]:
                trans[:3][:,3][:2] = w[-2:]
                robot.SetTransform(trans)
                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_reset)
    else:
        if premotion:
            gripper_before()
            for w in waypts[::-1][1:]:
                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_reset)

        for i in range(reps):
            # Attempt
            for w in waypts:

                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_attempt)
            if premotion:
                gripper_after()

            time.sleep(pause_btwn)

            if premotion and i == reps-1:
                break
            if premotion:
                gripper_before()

            # Reset
            for w in waypts[::-1][1:]:
                #print w
                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_reset)


def interpolate(start, goal, num_waypts):
    """
    Linear interpolation from start to goal in C-space.
    """
    init_waypts = np.zeros((num_waypts,7))
    for count in range(num_waypts):
        init_waypts[count,:] = start + count/(num_waypts - 1.0)*(goal - start)
    return init_waypts


def pre_motion_traj(premotion_starting_config, attempt_starting_config, \
                    joint_vel_coeff=20, collision_coeff=20, init_traj_input=None):
    #global n_steps, Final_pose, manip, ideal_config, ideal_config_vanilla
    n_steps = 30

    robot.SetDOFValues(attempt_starting_config, manip.GetArmIndices())
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink('r_gripper_palm_link'))
    linkstrans = robot.GetLinkTransformations()
    xyz_target1 = list(linkstrans[link_idx][:3][:,3]) #get xyz of specific link
    quat_target1 = list(Quaternion(matrix=linkstrans[link_idx]))
    robot.SetDOFValues(premotion_starting_config, manip.GetArmIndices())

    init_traj = interpolate(premotion_starting_config, attempt_starting_config, n_steps)

    #filling in request dictionary
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "rightarm",
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : [joint_vel_coeff]}
        },
        {
            "type" : "collision",
            "params" : {
                "coeffs" : [collision_coeff], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
              "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
                }   
        }, 

        ],
        "constraints" : [{"type": "pose",
                    "params" : {
                    "xyz" : xyz_target1, 
                    "wxyz" : quat_target1, 
                    "link": "r_gripper_palm_link",
                    "timestep" : n_steps-1,
                    "rot_coeffs" : [2,2,2],
                    "pos_coeffs" : [5,5,5]
                    }},

                      {
                        "type" : "joint", # joint-space target
                        "params" : {"vals" : attempt_starting_config.tolist() } # length of vals = # dofs of manip
                      }],
        "init_info" : {
           #"type": "given_traj",
           #"data": init_traj.tolist()
           #"type": "straight_line",
           #"endpoint": attempt_starting_config.tolist()
           #"type": "stationary",
        }   
    }

    if init_traj_input is None:
        request["init_info"]["type"] = "straight_line"
        request["init_info"]["endpoint"] = attempt_starting_config.tolist()
    else:
        request["init_info"]["type"] = "given_traj"
        request["init_info"]["data"] = init_traj_input.tolist()


    s = json.dumps(request)
    with openravepy.RobotStateSaver(robot):
      with util.suppress_stdout():
        prob = trajoptpy.ConstructProblem(s, robot.GetEnv()) # create object that stores optimization problem
        result = trajoptpy.OptimizeProblem(prob) # do optimization

    traj = result.GetTraj()
    dof_inds = sim_util.dof_inds_from_name(robot, manip_name)
    sim_util.unwrap_in_place(traj, dof_inds)
    
    return traj, result


def premotion(premotion_starting_config, attempt_starting_config, **kwargs):
    robot.SetDOFValues(premotion_starting_config, manip.GetArmIndices())
    pre_motion = pre_motion_traj(premotion_starting_config, attempt_starting_config, **kwargs)
    return pre_motion

def difference_in_traj(traj):
    t=1
    difference=[]
    sum_diff = []
    while t < len(traj):
        difference.append(np.abs(traj[t][:7]-traj[t-1][:7]))
        t=t+1
    for d in difference:
        sum_diff.append(sum(d))
    sum_diff = np.array(sum_diff)
    return difference, sum_diff

def gripper_before():
    if TASK["name"]==params.LIFT["name"]:
        robot.SetDOFValues([1],[34]) 
    if TASK["name"]==params.PULL["name"]:
        robot.SetDOFValues([1],[34]) 
    if TASK["name"] == params.PULL_DOWN["name"]:
        robot.SetDOFValues([1],[34]) 
       

def gripper_after():
    if TASK["name"]==params.PULL["name"]:
        time.sleep(0.2)
        v =robot.GetDOFValues()
        v[robot.GetJoint('r_gripper_l_finger_joint').GetDOFIndex()] = 0.05
        robot.SetDOFValues(v)   
    if TASK["name"] == params.PULL_DOWN["name"]:
        time.sleep(0.2)
        v =robot.GetDOFValues()
        v[robot.GetJoint('r_gripper_l_finger_joint').GetDOFIndex()] = 0.1
        robot.SetDOFValues(v)             

def exclude_gripper_collisions(opposite=False, fingertips_only=False):
    cc = trajoptpy.GetCollisionChecker(env)
    links = []
    for lr in 'lr':
        if not fingertips_only:
            links.append(robot.GetLink("%s_wrist_flex_link" % (lr)))
            links.append(robot.GetLink("%s_wrist_roll_link" % (lr)))
            links.append(robot.GetLink("%s_gripper_palm_link" % (lr)))
        for flr in 'lr':
            links.append(robot.GetLink("%s_gripper_%s_finger_tip_link" % (lr, flr)))
            if not fingertips_only:
                links.append(robot.GetLink("%s_gripper_%s_finger_link" % (lr, flr)))

    for r_link in links:
        for kin_body in kin_bodies:
            for link in kin_body.GetLinks():
                if not opposite:
                    cc.ExcludeCollisionPair(r_link, link)
                else:
                    cc.IncludeCollisionPair(r_link, link)

def execute_full_traj(start, attempt_speed, reset_speed, base, pre_motion_traj, traj, pause_btwn=0.05, only_premotion=False):
    robot.SetDOFValues(params.right_arm_attitude, manip.GetArmIndices()) #set rightarm to initial resting position
    gripper_before()
    executeArm(pre_motion_traj, params.right_arm_attitude, attempt_speed=2.0)
    gripper_after()
    time.sleep(0.3)
    if not only_premotion:
        orig_traj = np.copy(traj)
        traj = trim(traj)
        executeBothTimed(traj, start, attempt_speed=attempt_speed, reset_speed=reset_speed,both=base, pause_btwn=pause_btwn)
        #diff,sumd = difference_in_traj(traj)

def execute_heuristic_baseline(start, attempt_speed, reset_speed, base, pre_motion_traj, rewind_t=10, pause_btwn=0.05):
    # Executes back-and-forth heuristic baseline proposed in Kobayashi and Yamada, "Informing a User of a Robot's Mind by Motion"
    # Robot moves to failure point (i.e., executes pre_motion_traj), then rewinds
    # for last rewind_t steps of pre_motion_traj, executes that again, and repeats
    # this for a total of 3 times
    print "length of pre_motion_traj:", len(pre_motion_traj)
    robot.SetDOFValues(params.right_arm_attitude, manip.GetArmIndices()) #set rightarm to initial resting position
    gripper_before()
    executeArm(pre_motion_traj, params.right_arm_attitude, attempt_speed=2.0)
    gripper_after()
    time.sleep(0.3)
    orig_traj = np.copy(pre_motion_traj)
    traj = trim(pre_motion_traj)[-rewind_t:]
    executeBothTimed(traj, start, attempt_speed=attempt_speed, reset_speed=reset_speed, both=base, pause_btwn=pause_btwn, premotion=True)

def main():
    global BASE
    if (TASK["name"]==params.LIFT["name"]  or TASK["name"]==params.PULL_DOWN["name"]):
        BASE = False

    if EXCLUDE_GRIPPER_COLLISIONS_FOR_ATTEMPT:
        exclude_gripper_collisions()

    gripper_before()
    s,g,traj,trajs,result, costs, idx, results = best_starting_config() #calculate attempt traj

    if EXCLUDE_GRIPPER_COLLISIONS_FOR_ATTEMPT:
        exclude_gripper_collisions(opposite=True)

    exclude_gripper_collisions(fingertips_only=True)
    pre_motion_traj_smooth, pre_motion_result_smooth = \
            premotion(params.right_arm_attitude, s, joint_vel_coeff=200)

    execute_full_traj(s, FAST, MEDIUM, BASE, pre_motion_traj_smooth, traj)

    import IPython as ipy
    ipy.embed()


if __name__ == "__main__":
    main()


#look compare base cost and compare elbow cost to cost of stationary trajectory,
#and don't consider trajectory if either one of them is larger than the stationary trajectory cost 
#make new params for new starting params 

