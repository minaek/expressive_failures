import json, numpy as np, openravepy, time, trajoptpy

import params
from pyquaternion import Quaternion
from lfd.environment import sim_util
from lfd.util import util

FAST = 2.7
MEDIUM = 0.9
SLOW = 0.3

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

def trim(waypts):
    # Deletes redundant waypoints
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

def execute_both_timed(shared, waypts, starting_config, reps=3, \
                     attempt_speed=1.0, reset_speed=0.2, both=True, pause_btwn=0.05):
    # attempt_speed, reset_speed - in units of change in configuration space per second

    robot = shared.robot
    manip = shared.manip
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

    if both==True:
        for _ in range(reps):
            # Attempt
            for w in waypts:

                trans[:3][:,3][:2] = w[-2:]
                robot.SetTransform(trans)
                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_attempt)

            #time.sleep(0.1)
            time.sleep(pause_btwn)

            # Reset
            for w in waypts[::-1][1:]:
                trans[:3][:,3][:2] = w[-2:]
                robot.SetTransform(trans)
                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_reset)
    else:
        for _ in range(reps):
            # Attempt
            for w in waypts:

                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_attempt)

            #time.sleep(0.1)
            time.sleep(pause_btwn)

            # Reset
            for w in waypts[::-1][1:]:
                #print w
                robot.SetDOFValues(w[:7],manip.GetArmIndices())
                time.sleep(t_reset)

def execute_traj(shared, start, traj, attempt_speed=FAST, reset_speed=MEDIUM, pause_btwn=0.05):
    robot = shared.robot
    manip = shared.manip
    gripper_after(shared)
    robot.SetDOFValues(start, manip.GetArmIndices()) #set rightarm to initial resting position
    time.sleep(0.3)
    orig_traj = np.copy(traj)
    traj = trim(traj)
    execute_both_timed(shared, traj, start, attempt_speed=attempt_speed, reset_speed=reset_speed, both=shared.use_base, pause_btwn=pause_btwn)

def exclude_gripper_collisions(shared, opposite=False, fingertips_only=False):
    cc = trajoptpy.GetCollisionChecker(shared.env)
    links = []
    for lr in 'lr':
        if not fingertips_only:
            links.append(shared.robot.GetLink("%s_wrist_flex_link" % (lr)))
            links.append(shared.robot.GetLink("%s_wrist_roll_link" % (lr)))
            links.append(shared.robot.GetLink("%s_gripper_palm_link" % (lr)))
        for flr in 'lr':
            links.append(shared.robot.GetLink("%s_gripper_%s_finger_tip_link" % (lr, flr)))
            if not fingertips_only:
                links.append(shared.robot.GetLink("%s_gripper_%s_finger_link" % (lr, flr)))

    for r_link in links:
        for kin_body in shared.kin_bodies:
            for link in kin_body.GetLinks():
                if not opposite:
                    cc.ExcludeCollisionPair(r_link, link)
                else:
                    cc.IncludeCollisionPair(r_link, link)

def gripper_before(shared):
    if shared.task["name"] in [params.LIFT["name"], params.PULL["name"], params.PULL_DOWN["name"]]:
        shared.robot.SetDOFValues([1],[34])

def gripper_after(shared):
    robot = shared.robot
    if shared.task["name"] == params.PULL["name"]:
        time.sleep(0.2)
        v = robot.GetDOFValues()
        v[robot.GetJoint('r_gripper_l_finger_joint').GetDOFIndex()] = 0.05
        robot.SetDOFValues(v)   
    elif shared.task["name"] == params.PULL_DOWN["name"]:
        time.sleep(0.2)
        v =robot.GetDOFValues()
        v[robot.GetJoint('r_gripper_l_finger_joint').GetDOFIndex()] = 0.1
        robot.SetDOFValues(v)             

def position_base_request(shared, cost_fn_xsg, starting_config, n_steps, init_position=[0,0]):
    # init_position - initial position of robot's base
    # seems like nsteps needs to be defined as 1 when the base is the only active DOF initialized

    # Initialize robot position
    T = shared.robot.GetTransform()
    T[:,3][:2] = init_position
    shared.robot.SetTransform(T)
    gripper_before(shared)

    goal_config = shared.goal_config_stationary
    armids = list(shared.manip.GetArmIndices()) #get arm indices
    shared.robot.SetDOFValues(starting_config, armids)
    links = shared.robot.GetLinks()
    link_idx = links.index(shared.robot.GetLink('r_gripper_palm_link'))
    linkstrans = shared.robot.GetLinkTransformations()
    xyz_target = list(linkstrans[link_idx][:3][:,3]) #get xyz of specific link
    quat_target = list(Quaternion(matrix=linkstrans[link_idx])) #get quat of specific link
    if shared.use_base:
        shared.robot.SetActiveDOFs(np.r_[shared.manip.GetArmIndices()], openravepy.DOFAffine.X + openravepy.DOFAffine.Y)

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
    request["basic_info"]["manip"] = "active" if shared.use_base else "rightarm"

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
    prob = trajoptpy.ConstructProblem(s, shared.env)

    cost_fn = lambda x: cost_fn_xsg(x, starting_config, shared.goal_config_stationary)
    with openravepy.RobotStateSaver(shared.robot):
      with util.suppress_stdout():
        n_dof = 7
        if shared.use_base:
            n_dof = 9
        prob.AddCost(cost_fn, [(n_steps-1,j) for j in range(n_dof)], "express%i"%(n_steps-1))
        result = trajoptpy.OptimizeProblem(prob)
    traj = result.GetTraj()
    dof_inds = sim_util.dof_inds_from_name(shared.robot, shared.manip_name)
    sim_util.unwrap_in_place(traj, dof_inds)
    return traj, result
