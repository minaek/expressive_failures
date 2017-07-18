import numpy as np, random
from lfd.util import util
from lfd.rapprentice import math_utils as mu, resampling, retiming
from lfd.transfer import planning
from lfd.environment import sim_util

def open_gripper(pr2, lr):
    gripper = {"l":pr2.lgrip, "r":pr2.rgrip}[lr]
    gripper.open()

def close_gripper(pr2, lr):
    gripper = {"l":pr2.lgrip, "r":pr2.rgrip}[lr]
    #gripper.close()
    gripper.set_angle(0.045)  # So the cup doesn't slip out of the gripper's grasp

def move_gripper(sim, t_end, R_end=None, lr='r', n_steps=10, \
                 animation=0, interactive=0, beta_rot=1000.0, \
                 grasp_cup=False, R=0.02, D=0.035, cup_xyz=None):
    random.seed(1)
    ee_link_name = "%s_gripper_tool_frame"%lr
    hmat_start = sim.robot.GetLink(ee_link_name).GetTransform()
    if R_end is None:
        if lr == 'r':
            theta = np.pi/2
        elif lr == 'l':
            theta = -np.pi/2
        else:
            raise
        R_end = util.rotation_z(theta)
    hmat_end = np.r_[np.c_[R_end, t_end], np.c_[0,0,0,1]]
    traj, dof_inds, pose_costs = \
            plan_full_traj(sim.robot, hmat_start, hmat_end, True, lr=lr, \
                           beta_rot=beta_rot, n_steps=n_steps, \
                           grasp_cup=grasp_cup, R=R, D=D, cup_xyz=cup_xyz)
    full_traj = (traj, dof_inds)

    if sim.viewer:
        sim_callback = lambda i: sim.viewer.Step()
    else:
        sim_callback = lambda i: None

    # Execute the trajectory in OpenRave (not in Gazebo)
    sim.execute_trajectory(full_traj, step_viewer=animation, \
                           interactive=interactive, sim_callback=sim_callback, \
                           max_cart_vel_trans_traj=.05, max_cart_vel=.05)
    return full_traj, pose_costs

def plan_full_traj(robot, hmat_start, hmat_end, start_fixed, n_steps=10, \
                   lr='r', beta_rot=1000.0, grasp_cup=False, R=0.02, D=0.035, \
                   cup_xyz=None):
    manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
    ee_link_name = "%s_gripper_tool_frame"%lr
    robot.SetActiveManipulator(manip_name)
    ee_link = robot.GetLink(ee_link_name)

    new_hmats = np.asarray(resampling.interp_hmats(np.arange(n_steps), \
                                  np.r_[0, n_steps-1], [hmat_start, hmat_end]))
    dof_vals = robot.GetDOFValues(robot.GetManipulator(manip_name).GetArmIndices())
    old_traj = np.tile(dof_vals, (n_steps,1))
    pose_weights = np.logspace(0, 1, num=n_steps, base=10) / 10

    # pose_costs = (rotation squared err, position squared error)
    traj, _, pose_costs = \
            planning.plan_follow_traj(robot, manip_name, ee_link, new_hmats,
                                      old_traj, start_fixed=start_fixed,
                                      beta_rot=beta_rot, beta_pos=100000.0, \
                                      pose_weights=pose_weights, gamma=1000, \
                                      grasp_cup=grasp_cup, R=R, D=D, cup_xyz=cup_xyz)
    dof_inds = sim_util.dof_inds_from_name(robot, manip_name)
    return traj, dof_inds, pose_costs

def follow_body_traj(pr2, bodypart2traj, wait=True, \
                     base_frame = "/base_footprint", speed_factor=1):    
    name2part = {"lgrip":pr2.lgrip, 
                 "rgrip":pr2.rgrip, 
                 "larm":pr2.larm, 
                 "rarm":pr2.rarm}
    for partname in bodypart2traj:
        if partname not in name2part:
            raise Exception("invalid part name %s"%partname)
            
    #### Go to initial positions #######
    for (name, part) in name2part.items():
        if name in bodypart2traj:
            part_traj = bodypart2traj[name]        
            if name == "lgrip" or name == "rgrip":
                part.set_angle(np.squeeze(part_traj)[0])
            elif name == "larm" or name == "rarm":
                part.goto_joint_positions(part_traj[0])
    pr2.join_all()

    #### Construct total trajectory so we can retime it #######
    n_dof = 0            
    trajectories = []
    vel_limits = []
    acc_limits = []
    bodypart2inds = {}
    for (name, part) in name2part.items():
        if name in bodypart2traj:
            traj = bodypart2traj[name]
            if traj.ndim == 1: traj = traj.reshape(-1,1)
            trajectories.append(traj)
            vel_limits.extend(part.vel_limits)
            acc_limits.extend(part.acc_limits)
            bodypart2inds[name] = range(n_dof, n_dof+part.n_joints)
            n_dof += part.n_joints
                        
    trajectories = np.concatenate(trajectories, 1)    
    vel_limits = np.array(vel_limits)*speed_factor
    print "VEL LIMITS:", vel_limits, speed_factor

    times = retiming.retime_with_vel_limits(trajectories, vel_limits)
    times_up = np.linspace(0, times[-1], int(np.ceil(times[-1]/.1)))
    traj_up = mu.interp2d(times_up, times, trajectories)
    
    #### Send all part trajectories ###########
    for (name, part) in name2part.items():
        if name in bodypart2traj:
            part_traj = traj_up[:,bodypart2inds[name]]
            if name == "lgrip" or name == "rgrip":
                part.follow_timed_trajectory(times_up, part_traj.flatten())
            elif name == "larm" or name == "rarm":
                vels = resampling.get_velocities(part_traj, times_up, .001)
                part.follow_timed_joint_trajectory(part_traj, vels, times_up)
            elif name == "base":
                part.follow_timed_trajectory(times_up, part_traj, base_frame)
                
    if wait: pr2.join_all()    
    pr2.update_rave()
    return True
