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
import trajoptpy
from lfd.util import util
import time
from math import *
import cost
from cost import *
from pyquaternion import Quaternion
import globalvars
from lfd.environment import sim_util

COST_FN_XSG = lambda x,s,g: cost_projections(x, s, g, d=3, coeff=20)
#COST_FN_XSG = lambda x,s,g: cost_distance_bet_deltas(x, s, g, coeff=20)

COST_FN_BASE = lambda x, s, g: base(x,s,g,d=3, coeff=20)

N_STEPS=10

def best_starting_config(): 
    """
    Optimizes for the best starting configuration.

    returns starting configuration, goal configuration, and trajectory
    """
    global sols,starting_configs,goal_config_stationary,Tgoal
    trajs = [] #list of lists
    costs = [] #list of lists
    results = [] #contains result from solving TrajOpt problems
    print "len of sols: " + str(len(sols))
    for s in range(len(sols)):
        print s
        candidate_attempt_traj, result = position_base_request(starting_configs[s],goal_config_stationary)
        trajs.append(candidate_attempt_traj) #append trajectory for a given ik solution
        results.append(result)
        waypt_costs = [] 
        for waypt in candidate_attempt_traj:  
            waypt_costs.append(COST_FN_XSG(waypt, starting_configs[s], goal_config_stationary))
            waypt_costs.append(COST_FN_BASE(waypt, starting_configs[s], goal_config_stationary))

        costs.append(waypt_costs) #appending list of costs

    idx = np.argmin([sum(x) for x in costs]) #find the trajectory with the lowest cost sum
    #print "COSTS: " + str(costs)
    #print "idx: " + str(idx)
    #print "smallest vals: " + str(costs[idx])
    traj = trajs[idx]
    starting_config = starting_configs[idx]

    return starting_config, goal_config_stationary, traj, trajs

def optimize_starting_and_cost():
    global sols,starting_configs,goal_config_stationary,Tgoal, COST_FN_XSG, COST_FN_BASE
    trajs = [] #list of lists
    costs = [] #list of lists
    print "len of sols: " + str(len(sols))
    for s in range(len(sols)):
        print s
        for c in range(1,21):
            COST_FN_XSG = lambda x,s,g: cost_projections(x,s,g,d=3,coeff=c) #update cost functions 
            COST_FN_BASE=lambda x,s,g: base(x,s,g,d=3,coeff=c)
            candidate_attempt_traj = position_base_request(starting_configs[s],goal_config_stationary)
            trajs.append(candidate_attempt_traj) #append trajectory for a given ik solution
            waypt_costs = [] 
            for waypt in candidate_attempt_traj:  
                waypt_costs.append(COST_FN_XSG(waypt, starting_configs[s], goal_config_stationary,d=3,coeff=c))
            costs.append(waypt_costs) #appending list of costs

def position_base_request(starting_config, goal_config, init_position=[0,0]):
    global robot
    n_steps = N_STEPS #seems like nsteps needs to be defined as 1 when the base is the only active DOF initialized
    T = robot.GetTransform()
    T[:,3][:2] = init_position
    robot.SetTransform(T)
    robot.SetDOFValues(starting_config, manip.GetArmIndices())
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink('r_gripper_palm_link'))
    linkstrans = robot.GetLinkTransformations()
    xyz_target = list(linkstrans[link_idx][:3][:,3]) #get xyz of specific link
    quat_target = list(Quaternion(matrix=linkstrans[link_idx])) #get quat of specific link
    robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices()], DOFAffine.X + DOFAffine.Y)

    #robot.SetActiveDOFs([], DOFAffine.X + DOFAffine.Y) #set base as only active dof
    #robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices()])
    request = {
        # BEGIN basic_info
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "active",
            #'manip': 'rightarm',
            "start_fixed" : True  # DOF values at first timestep are fixed based on current robot state
        },
        # END basic_info
        "costs" : [
        {
            "type" : "joint_vel",
            "params" : {"coeffs" : [1]}
        }
        ],
        "constraints" : [
        #{
            #"type" : "pose",
            #"name" : "final_pose",
            #"params" : {
            #    "pos_coeffs" : [6,6,6],
            #    "rot_coeffs" : [2,2,2],
            #    "xyz" : list(xyz_target),
            #    "wxyz" : list(quat_target),
            #    "link" : "r_gripper_palm_link",
            #},
        #}
        ],
        "init_info" : {
        }
    }

    ##Initialize trajectory as stationary##:
    request["init_info"]["type"] = "stationary"

    for i in range(n_steps):
        request["constraints"].append({
            "type" : "pose",
            "name" : "pose"+str(i),
            "params" : {
                "pos_coeffs" : [6,6,6],
                "rot_coeffs" : [2,2,2],
                "xyz" : list(xyz_target),
                "wxyz" : list(quat_target),
                "link" : "r_gripper_palm_link",
                "timestep": i,
            }})

    ##This is similar to how the online example initializes the traj##:
    # dofvals_init = robot.GetActiveDOFValues()
    # print len(dofvals_init)
    # we'll treat the base pose specially, choosing a random angle and then setting a reasonable
    # position based on this angle
    # angle_init = np.random.rand() * 2*np.pi
    # x_init = xyz_target[0] - .5*np.cos(angle_init)
    # y_init = xyz_target[1] - .5*np.sin(angle_init)    
    # dofvals_init[-3:] = [x_init, y_init, angle_init]
    # END random_init
    #robot.SetDOFValues(starting_config, manip.GetArmIndices())
    # request["init_info"]["type"] = "given_traj"
    # request["init_info"]["data"] = [dofvals_init.tolist()]


    s = json.dumps(request)
    prob = trajoptpy.ConstructProblem(s, env)

    cost_fn = lambda x: COST_FN_XSG(x, starting_config, goal_config_stationary)
    cost_fn2  = lambda x: COST_FN_BASE(x, starting_config, goal_config_stationary)
    #for n in range(n_steps):
    #    prob.AddCost(cost_fn2, [(n,j) for j in range(7)], "base%i"%(n))

    prob.AddCost(cost_fn, [(n_steps-1,j) for j in range(7)], "table%i"%(n_steps-1))
    prob.AddCost(cost_fn2, [(n_steps-1,j) for j in range(7,9)], "base%i"%(n_steps-1))
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

def executeArm(waypts, reps=3, t=0.1):
    """
    Executes in the planned trajectory in simulation
    """
    global env,robot,manip
    
    #waypts = np.insert(waypts,0,starting_config,axis=0)
    #trajs = [waypts, np.flip(waypts,0)]
    for _ in range(reps):
        for w in waypts:
            robot.SetDOFValues(w[:7],manip.GetArmIndices())
            time.sleep(0.1)

def executeBase(waypts, reps = 3, t=0.1):
    global robot
    trans = robot.GetTransform()
    for _ in range(reps):
        for w in waypts:
            trans[:3][:,3][:2] = w[-2:]
            print w[-2:]
            robot.SetTransform(trans)
            time.sleep(t)

def executeBoth(waypts, starting_config, reps=3, t=0.1):
    global robot
    print "First waypt starts at base location:", waypts[0][-2:]
    trans = robot.GetTransform()
    for _ in range(reps):
        for w in waypts:
            trans[:3][:,3][:2] = w[-2:]
            robot.SetTransform(trans)
            robot.SetDOFValues(w[:7],manip.GetArmIndices())
            time.sleep(t)



def executeBothTimed(waypts, starting_config, reps=3, \
                     attempt_speed=1.0, reset_speed=0.2):
    # attempt_speed, reset_speed - in units of change in configuration space per second

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

    for _ in range(reps):
        # Attempt
        for w in waypts:
            trans[:3][:,3][:2] = w[-2:]
            robot.SetTransform(trans)
            robot.SetDOFValues(w[:7],manip.GetArmIndices())
            time.sleep(t_attempt)
        time.sleep(0.1)
        # Reset
        for w in waypts[::-1][1:]:
            trans[:3][:,3][:2] = w[-2:]
            robot.SetTransform(trans)
            robot.SetDOFValues(w[:7],manip.GetArmIndices())
            time.sleep(t_reset)


def calculate_EE_movement(traj):
    deviation=0
    for waypt in traj:
        robot.SetDOFValues(waypt, manip.GetArmIndices())
        EE_pos = manip.GetEndEffectorTransform()[:3][:,3]
        EE_goal_pos = Tgoal[:3][:,3]
        deviation+=np.abs(np.linalg.norm(EE_goal_pos-EE_pos))
    return deviation/N_STEPS


def main():
    success = False
    s,g,traj,trajs = best_starting_config()
    deviation = calculate_EE_movement(traj)
    print "EE deviation: " + str(deviation)
    executeBothTimed(traj, s, attempt_speed=0.3, reset_speed=0.9)

    # for i_try in xrange(100):
    #     request= position_base_request()
    #     s = json.dumps(request)
    #     prob = trajoptpy.ConstructProblem(s, env)
    #     cost_fn = lambda x: COST_FN_XSG(x, starting_config, goal_config_stationary)
    #     prob.AddCost(cost_fn, [(n_steps-1,j) for j in range(7)], "table%i"%(n_steps-1))
    #     result = trajoptpy.OptimizeProblem(prob)
    #     if check_result(result, robot): 
    #         success = True
    #         break
    # traj = result.GetTraj()


    #executePathSim(traj)
    #robot.SetDOFValues(starting_config, manip.GetArmIndices())

    import IPython as ipy
    ipy.embed()
            
    # if success:
    #     print "succeeded on try %i"%(i_try)
    #     print result
    # else:
    #     print "failed to find a valid solution :("

if __name__ == "__main__":
    main()
 

#EE deviation: 0.333814612829
#EE deviation: 3.82393223776
#0.0764786447552 N = 50
#EE deviation: 1.29652855485e-09, N=100, does not move

#EE deviation: 0.0792912272914, N=100, cost function coefficients = 10 

#0.0777673460262 N=100, cost function coefficients = 10 
