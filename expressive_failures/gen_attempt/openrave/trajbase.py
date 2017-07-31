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


def position_base_request():
    global starting_config, robot, goal_config, manip
    n_steps = 10 #seems like nsteps needs to be defined as 1 when the base is the only active DOF initialized
    robot.SetDOFValues(starting_config, manip.GetArmIndices())
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink('r_gripper_palm_link'))
    linkstrans = robot.GetLinkTransformations()
    xyz_target = list(linkstrans[link_idx][:3][:,3]) #get xyz of specific link
    quat_target = list(Quaternion(matrix=linkstrans[link_idx])) #get quat of specific link
    robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices()], #set arm and base as active dofs
                   DOFAffine.X + DOFAffine.Y + DOFAffine.RotationAxis, [0,0,1])
    #robot.SetActiveDOFs([], DOFAffine.X + DOFAffine.Y + DOFAffine.RotationAxis, [0,0,1]) #set base as only active dof
    request = {
        # BEGIN basic_info
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "active",
            "start_fixed" : False
        },
        # END basic_info
        "costs" : [
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.025]}
        },
        {
            "type" : "joint_vel",
            "params" : {"coeffs" : [1]}
        }
        ],
        "constraints" : [
        {
            "type" : "pose",
            "name" : "final_pose",
            "params" : {
                "pos_coeffs" : [5,5,5],
                "rot_coeffs" : [2,2,2],
                "xyz" : list(xyz_target),
                "wxyz" : list(quat_target),
                "link" : "r_gripper_palm_link",
            },
        }
        ],
        "init_info" : {
        }
    }

    #Initialize trajectory like this
    request["init_info"]["type"] = "stationary"

    #OR

    #Initialize trajectory the way the example online does it
    # BEGIN random_init
    # randomly select joint values with uniform distribution over joint limits 
    # lower,upper = robot.GetActiveDOFLimits()
    # print len(lower)
    # lower = np.clip(lower, -np.pi, np.pi) # continuous joints have huge ranges, so we clip them to [-pi, pi]
    # upper = np.clip(upper, -np.pi, np.pi) # to avoid poor numerical conditioning
    # rands = np.random.rand(len(lower))
    # dofvals_init2 = lower*rands + upper*(1-rands)
    # print len(dofvals_init2), len(lower)

    dofvals_init = robot.GetActiveDOFValues()
    print len(dofvals_init)
    # we'll treat the base pose specially, choosing a random angle and then setting a reasonable
    # position based on this angle
    angle_init = np.random.rand() * 2*np.pi
    x_init = xyz_target[0] - .5*np.cos(angle_init)
    y_init = xyz_target[1] - .5*np.sin(angle_init)    
    dofvals_init[-3:] = [x_init, y_init, angle_init]
    # END random_init
    #robot.SetDOFValues(starting_config, manip.GetArmIndices())
    # request["init_info"]["type"] = "given_traj"
    # request["init_info"]["data"] = [dofvals_init.tolist()]
    
    return request, dofvals_init

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
    global starting_config,manip, goal_config_stationary
    robot.SetDOFValues(starting_config, manip.GetArmIndices())
    success = False
        
    for i_try in xrange(100):
        request,init = position_base_request()
        s = json.dumps(request)
        # trajoptpy.SetInteractive(args.interactive)
        prob = trajoptpy.ConstructProblem(s, env)
        cost_fn = lambda x: COST_FN_XSG(x, starting_config, goal_config_stationary)
        prob.AddCost(cost_fn, [(n_steps-1,j) for j in range(7)], "table%i"%(n_steps-1))
        result = trajoptpy.OptimizeProblem(prob)
        if check_result(result, robot): 
            success = True
            break
    traj = result.GetTraj()
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
 
