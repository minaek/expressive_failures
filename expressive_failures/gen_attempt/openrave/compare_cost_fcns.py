#!/usr/bin/env python

import h5py
import itertools
import numpy as np
import openravepy

from shared_costs import cost_projections, cost_match_positions
from shared_vars import SharedVars
from shared_util import exclude_gripper_collisions, position_base_request, execute_traj
import params

N_STEPS = 20

# Options for cost functions and parameters (cost coefficient, alpha)
COST_FNS = [("cost_match_pos", lambda *args, **kw: cost_match_positions(*args, **kw)),
            ("cost_proj_d1", lambda *args, **kw:   cost_projections(*args, d=1, **kw)),
            ("cost_proj_d3", lambda *args, **kw:   cost_projections(*args, d=3, **kw)),
            ("cost_proj_d5", lambda *args, **kw:   cost_projections(*args, d=5, **kw)),
            ("cost_proj_d7", lambda *args, **kw:   cost_projections(*args, d=7, **kw)),
            ]
#COST_COEFFS = [1,2,5,10,20,40,80]  # TODO: Change back
#ALPHAS = [0,0.3,0.6,1.0,2.0,4.0,8.0]  # TODO: Change back
COST_COEFFS = [20]
ALPHAS = [0.0, 0.3,1.0,10.0]

# TODO: Change this depending on the cost function?
LINK_NAMES = ['r_elbow_flex_link', 'r_shoulder_lift_link', 'torso_lift_motor_screw_link']

# Tasks to compute expressive failure trajectories for
#TASKS = [params.LIFT, params.PUSH, params.PULL, params.PULL_DOWN, params.PUSH_SIDEWAYS]
TASKS = [params.PUSH_SIDEWAYS, params.PULL, params.PULL_DOWN, params.PUSH, params.LIFT]

EXCLUDE_GRIPPER_COLLISIONS_FOR_ATTEMPT = True

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
    for i,start_config in enumerate(shared.starting_configs):
        print "\t",i
        candidate_attempt_traj, result = position_base_request(shared, cost_fn_xsg, start_config, N_STEPS)
        trajs.append(candidate_attempt_traj)  # append trajectory for a given ik solution for q_s
        cost = np.sum([val for x,val in result.GetCosts() if x=="f"])  # total cost

        # check to see whether pose constraints were violated at any timestep
        pose_violation = np.max([val for name,val in result.GetConstraints() if "pose" in name])
        if pose_violation > 1e-3:
            cost += 1000
        
        # check to see whether collision costs are zero
        coll_cost = np.sum([y for x,y in result.GetCosts() if x.startswith("collision")])
        if coll_cost > 1e-3:
            cost += 10000
        costs.append(cost)

    #import IPython as ipy
    #ipy.embed()
    idx = np.argmin(costs) #find the trajectory with the lowest cost sum
    execute_traj(shared, shared.starting_configs[idx], trajs[idx])
    return shared.starting_configs[idx], trajs[idx], costs[idx]

def standardize_cost(cost, cost_params):
    # cost = coeff*(alpha + x)
    # x = (cost / coeff) - alpha
    coeff = cost_params["coeff"]
    alpha = cost_params["alpha"]
    return cost / float(coeff) - alpha

def compute_expressive_failure(shared, cost_name, cost_fn, link_names, cost_params, output_h5):
    costs = []
    for cost_p in cost_params:
        print "Cost params:", cost_p
        cost_fn_xsg = lambda x,s,g: cost_fn(shared, x, s, g, link_names, **cost_p)
        start_config, traj, cost = best_starting_config(shared, cost_fn_xsg)
        cost = standardize_cost(cost, cost_p)
        costs.append((cost, cost_p, start_config, traj))

    print costs

    # Save output
    f = h5py.File(output_h5, 'r+')
    if shared.task["name"] not in f:
        f.create_group(shared.task["name"])
    g = f[shared.task["name"]].create_group(cost_name)
    g.create_group("all")
    for i, (cost, cost_p, start_config, traj) in enumerate(costs):
        g["all"].create_group(str(i))
        g["all"][str(i)]["cost"] = cost
        g["all"][str(i)].create_group("cost_params")
        for k,v in cost_p.items():
            g["all"][str(i)]["cost_params"][k] = v
        g["all"][str(i)]["cost_params"]["link_names"] = LINK_NAMES
        g["all"][str(i)]["start_config"] = start_config
        g["all"][str(i)]["traj"] = traj

    idx = np.argmin([cost for cost, cost_p, start_config, traj in costs])
    g.create_group("best")
    g["best"]["idx"] = idx
    g["best"]["cost"] = costs[idx][0]
    g["best"].create_group("cost_params")
    for k, v in costs[idx][1].items():
        g["best"]["cost_params"][k] = v
    g["best"]["cost_params"]["link_names"] = LINK_NAMES
    g["best"]["start_config"] = start_config
    g["best"]["traj"] = traj
    f.close()

def main():
    h5_fname = "output_" + get_time_stamp() + ".h5"
    f = h5py.File(h5_fname, 'w')
    f.close()
    cost_params = list(itertools.product(COST_COEFFS, ALPHAS))
    cost_params = [dict(coeff=coeff, alpha=alpha) for coeff, alpha in cost_params]
    for task in TASKS:
        print "Task:", task["name"]
        shared = SharedVars(task, env, robot)

        if EXCLUDE_GRIPPER_COLLISIONS_FOR_ATTEMPT:
            exclude_gripper_collisions(shared)

        for cost_name, cost_fn in COST_FNS:
            print "cost fcn:", cost_name
            compute_expressive_failure(shared, cost_name, cost_fn, LINK_NAMES, cost_params, h5_fname)

if __name__ == "__main__":
    env = openravepy.Environment()
    env.StopSimulation()
    env.SetViewer('qtcoin')
    robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
    env.AddRobot(robot)

    main()
