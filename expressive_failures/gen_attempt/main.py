from globalvars import *
from trajprac2 import *
import pr2_util
from lfd.demonstration import demonstration
import time
import numpy as np


init()
traj = attempt_traj()

print "visualizing attempt trajectory.."
lr = 'r'  # Execute task with just right gripper
D = 0.035  # distance from gripper point (fingertips when closed) to center
R = D * 0.67  # cup radius
PICK_UP_HEIGHT = 0.11  # 0.11
ADJ_D = D
cup_x=0.66 
cup_y=0
cup_z = TABLE_HEIGHT+0.03
TABLE_HEIGHT = 0.45
robot = globalvars.or_sim.env.GetRobots()[0]
manip = robot.GetActiveManipulator()
gripper_end = np.r_[R + ADJ_D*math.cos(np.pi/4), \
                    R + ADJ_D*math.sin(np.pi/4), \
                    cup_z+PICK_UP_HEIGHT]
#pr2_util.move_gripper(globalvars.or_sim, gripper_end, R_end=util.rotation_z(np.pi/4), lr=lr, \
#                         beta_rot=100000.0, n_steps=100, grasp_cup=True, R=R, \
#                         D=D, cup_xyz=np.r_[cup_x, cup_y, cup_z+PICK_UP_HEIGHT])
fulltraj = traj, manip.GetArmIndices().tolist()
if globalvars.pr2 is not None:

    aug_traj_tocup = \
            demonstration.AugmentedTrajectory.create_from_full_traj(globalvars.pr2.robot, fulltraj)
    bodypart2traj = {}
    for lr, arm_traj in aug_traj_tocup.lr2arm_traj.items():
        part_name = {"l":"larm", "r":"rarm"}[lr]
        bodypart2traj[part_name] = arm_traj
    pr2_util.follow_body_traj(globalvars.pr2, bodypart2traj, speed_factor=0.2)
    pr2_util.close_gripper(globalvars.pr2, lr)
    time.sleep(2)

    globalvars.or_sim.remove_objects([globalvars.cup])
    globalvars.pr2.update_rave()
else:
    print "in else statement"
    update_or_robot(robot, fulltraj[0][-1,:], rave_inds=fulltraj[1])

# move_to_initial_position(globalvars.pr2, robot)
time.sleep(1)