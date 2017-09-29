#!/usr/bin/env python

import h5py
import itertools
import numpy as np
import openravepy
import os, os.path as osp
import sys
import time

from shared_costs import cost_phi_ee, cost_phi_bodypts, cost_config
from shared_dists import dist_l2, dist_proj
from shared_vars import SharedVars
from shared_util import exclude_gripper_collisions, position_base_request, execute_traj
import params

N_STEPS = 20
MAX_ATTEMPTS = 5

# Options for distance fcn, cost fcn, and cost fcn parameters
DIST_FNS = [("dist_proj_k1", lambda v1,v2: dist_proj(v1,v2,k=1)),
            ("dist_proj_k3", lambda v1,v2: dist_proj(v1,v2,k=3)),
            ("dist_proj_k9", lambda v1,v2: dist_proj(v1,v2,k=9)),
            ("dist_l2",      dist_l2)
           ]

COST_FNS = [("cost_phi_ee", cost_phi_ee),
            ("cost_phi_bodypts", cost_phi_bodypts),
            ("cost_config", cost_config)
            ]

#COST_COEFFS = [1,5,10,20,40,80]
#ALPHAS = [0,0.3,0.6,1.0,2.0,4.0,8.0]

COST_COEFFS = [10,20,40,80,160]
ALPHAS = [0,0.3,0.6,1.0,2.0]
LINK_NAMES = ['r_elbow_flex_link', 'r_shoulder_lift_link', 'torso_lift_motor_screw_link']

# Tasks to compute expressive failure trajectories for
#TASKS = [params.PUSH, params.LIFT]
TASKS = [params.PUSH_SIDEWAYS, params.PULL, params.PULL_DOWN, params.PUSH, params.LIFT]

EXCLUDE_GRIPPER_COLLISIONS_FOR_ATTEMPT = True
JOINT_VEL_MAX = 1.0

CURRDIR = os.path.dirname(__file__)
LOGFILE = osp.join(CURRDIR, "log.txt")

def writelog(message):
    with open(LOGFILE,'a') as f:
        f.write("{0} {1}\n".format(time.asctime( time.localtime(time.time())),
                                   message))

def get_time_stamp():
    import datetime
    import dateutil.tz
    now = datetime.datetime.now(dateutil.tz.tzlocal())
    timestamp = now.strftime('%Y%m%d_%H%M%S_%f')
    return timestamp

def best_starting_config(shared, cost_fn_xsg):
    """
    Optimizes for the best starting configuration.

    returns starting configuration, goal configuration, and trajectory
    """
    trajs = [] #list of lists
    costs = [] #list of lists
    results = []
    for i,start_config in enumerate(shared.starting_configs):
        print "\t", i
        goal_config = shared.goal_configs[shared.start_goal_idx[i]]
        candidate_attempt_traj, result = position_base_request(shared, cost_fn_xsg, start_config, goal_config, N_STEPS)
        trajs.append(candidate_attempt_traj)  # append trajectory for a given ik solution for q_s
        results.append(result)
        cost = np.sum([val for x,val in result.GetCosts() if x=="f"])  # total cost

        # check to see whether joint velocity cost is too high
        joint_vel_cost = [val for x,val in result.GetCosts() if x=="joint_vel"]
        assert len(joint_vel_cost) == 1
        if joint_vel_cost[0] > JOINT_VEL_MAX:
            cost += 1000

        # check to see whether pose constraints were violated at any timestep
        pose_violation = np.max([val for name,val in result.GetConstraints() if "pose" in name])
        if pose_violation > 1e-3:
            cost += 10000
        
        # check to see whether collision costs are zero
        coll_cost = np.sum([y for x,y in result.GetCosts() if x.startswith("collision")])
        if coll_cost > 1e-3:
            cost += 100000
        costs.append(cost)

    idx = np.argmin(costs) #find the trajectory with the lowest cost sum
    #execute_traj(shared, shared.starting_configs[idx], trajs[idx])
    return idx, shared.starting_configs[idx], trajs[idx], costs[idx]

def standardize_cost(cost, cost_params):
    # cost = coeff*(alpha + x)
    # x = (cost / coeff) - alpha
    coeff = cost_params["coeff"]
    alpha = cost_params["alpha"]
    return cost / float(coeff) - alpha

def compute_expressive_failure(shared, cost_name, cost_fn, dist_name, dist_fn, link_names, cost_params, output_h5):
    costs = []
    for i, cost_p in enumerate(cost_params):
        # Check if this has already been computed
        computed = False
        f = h5py.File(output_h5, 'r')
        if shared.task["name"] in f and \
           cost_name in f[shared.task["name"]] and \
           dist_name in f[shared.task["name"]][cost_name] and \
           str(i) in f[shared.task["name"]][cost_name][dist_name]["all"]:
             if "cost" in f[shared.task["name"]][cost_name][dist_name]["all"][str(i)]:
                 computed = True
             elif f[shared.task["name"]][cost_name][dist_name]["all"][str(i)]["n_attempts"][()] > MAX_ATTEMPTS:
                 computed = True
        f.close()
        if computed:
            continue
        print "Cost params:", i, cost_p
        writelog(shared.task["name"] + " | " + cost_name + " | " + dist_name + \
                 " | " + str(i) + " " + str(cost_p))

        # Update output file
        f = h5py.File(output_h5, 'r+')
        if shared.task["name"] not in f:
            f.create_group(shared.task["name"])
            f[shared.task["name"]]["task_vector"] = shared.task["vector"]
        if cost_name not in f[shared.task["name"]]:
            f[shared.task["name"]].create_group(cost_name)
        if dist_name not in f[shared.task["name"]][cost_name]:
            g = f[shared.task["name"]][cost_name].create_group(dist_name)
            g.create_group("all")
        if str(i) not in f[shared.task["name"]][cost_name][dist_name]["all"]:
            f[shared.task["name"]][cost_name][dist_name]["all"].create_group(str(i))
        g = f[shared.task["name"]][cost_name][dist_name]["all"][str(i)]
        n_attempts = 1
        if "n_attempts" in g:
            n_attempts += g["n_attempts"][()]
            del g["n_attempts"]
        g["n_attempts"] = n_attempts
        f.close()
        if n_attempts > MAX_ATTEMPTS:  # Crashed too many times
            writelog("\tSkipping after " + str(MAX_ATTEMPTS) + " crashes")
            continue

        cost_fn_xsg = lambda x,s,g: cost_fn(shared, x, s, g, link_names, dist_fn, **cost_p)
        idx, start_config, traj, cost = best_starting_config(shared, cost_fn_xsg)
        cost = standardize_cost(cost, cost_p)
        costs.append((idx, cost, cost_p, start_config, traj))

        # Save output
        f = h5py.File(output_h5, 'r+')
        g = f[shared.task["name"]][cost_name][dist_name]["all"][str(i)]
        g["cost"] = cost
        g.create_group("cost_params")
        for k,v in cost_p.items():
            g["cost_params"][k] = v
        g["cost_params"]["link_names"] = LINK_NAMES
        g["start_config"] = start_config
        g["idx"] = idx
        g["traj"] = traj
        f.close()

    #idx = np.argmin([cost for idx, cost, cost_p, start_config, traj in costs])
    #g.create_group("best")
    #g["best"]["idx"] = idx
    #g["best"]["cost"] = costs[idx][0]
    #g["best"].create_group("cost_params")
    #for k, v in costs[idx][1].items():
    #    g["best"]["cost_params"][k] = v
    #g["best"]["cost_params"]["link_names"] = LINK_NAMES
    #g["best"]["start_config"] = start_config
    #g["best"]["traj"] = traj
    #f.close()

def main():
    #h5_fname = "output_" + get_time_stamp() + ".h5"
    #f = h5py.File(h5_fname, 'w')
    #f.close()

    #h5_fname = "output_20170927_220543_411510.h5"
    #h5_fname = "output_20170928_131100_000000.h5"
    #h5_fname = "output_20170928_185100_000000.h5"
    h5_fname = "output_20170929_105100_000000.h5"
    task_name = sys.argv[1]

    task = None
    for t in TASKS:
        if t["name"] == task_name:
            task = t
            break
    assert task is not None
    cost_params = list(itertools.product(COST_COEFFS, ALPHAS))
    cost_params = [dict(coeff=coeff, alpha=alpha) for coeff, alpha in cost_params]

    print "Task:", task["name"]
    shared = SharedVars(task, env, robot)

    if EXCLUDE_GRIPPER_COLLISIONS_FOR_ATTEMPT:
        exclude_gripper_collisions(shared)

    for cost_name, cost_fn in COST_FNS:
        print "cost fcn:", cost_name
        for dist_name, dist_fn in DIST_FNS:
            print "dist fcn:", dist_name
            compute_expressive_failure(shared, cost_name, cost_fn, dist_name, dist_fn, LINK_NAMES, cost_params, h5_fname)

    f = h5py.File(h5_fname, 'r+')
    if "done" not in f:
        f.create_group("done")
    f["done"][task["name"]] = 1
    f.close()

if __name__ == "__main__":
    env = openravepy.Environment()
    env.StopSimulation()
    env.SetViewer('qtcoin')
    robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
    env.AddRobot(robot)

    main()
