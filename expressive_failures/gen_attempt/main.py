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
robot = globalvars.or_sim.env.GetRobots()[0]
manip = robot.GetActiveManipulator()
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
    globalvars.or_sim.viewer.Step()  # Updates OpenRave viewer, to show new robot configuration

# move_to_initial_position(globalvars.pr2, robot)
time.sleep(1)
