from globalvars import *
from trajprac2 import *
import pr2_util
from lfd.demonstration import demonstration
import time
import numpy as np
import pickle
import os.path as osp
from lfd.util.util import get_time_stamp
from lfd.environment import sim_util
from expressive_failures.gen_attempt.demo import pr2_follow_traj, wait_until_done_moving

CACHE_DIR = osp.join(osp.dirname(osp.realpath(__file__)), "cache")
TRAJ_FILE = osp.join(CACHE_DIR, "traj.pkl")
#TRAJ_FILE = None
TRAJ_FILE_PREFIX = osp.join(CACHE_DIR, "cached_traj")

if TRAJ_FILE is not None and osp.dirname(osp.realpath(__file__)) not in TRAJ_FILE:
    TRAJ_FILE = osp.join(CACHE_DIR, TRAJ_FILE)

init()
if TRAJ_FILE is None:
    traj = attempt_traj()
    traj_fname = TRAJ_FILE_PREFIX + "_" + get_time_stamp() + ".pkl"
    pickle.dump(traj, open(traj_fname, 'wb'))
else:
    traj = pickle.load(open(TRAJ_FILE, 'rb'))
    traj = traj[:16]
    traj_back = np.copy(traj[::-1])
    robot = globalvars.or_sim.env.GetRobots()[0]
    manip_name = "rightarm"
    dof_inds = sim_util.dof_inds_from_name(robot, manip_name)
    #sim_util.unwrap_in_place(traj, dof_inds)

print "visualizing attempt trajectory.."
lr = 'r'  # Execute task with just right gripper
robot = globalvars.or_sim.env.GetRobots()[0]
manip = robot.GetActiveManipulator()
fulltraj = traj, manip.GetArmIndices().tolist()
fulltraj_back = traj_back, manip.GetArmIndices().tolist()
wait_until_done_moving(globalvars.pr2)

if globalvars.pr2 is not None:
    pr2_follow_traj(globalvars.pr2, fulltraj, speed_factor=4.0)
    pr2_follow_traj(globalvars.pr2, fulltraj_back, speed_factor=2.0)
    time.sleep(4)
    pr2_follow_traj(globalvars.pr2, fulltraj, speed_factor=4.0)
    pr2_follow_traj(globalvars.pr2, fulltraj_back, speed_factor=2.0)
    pr2_follow_traj(globalvars.pr2, fulltraj, speed_factor=4.0)
    pr2_follow_traj(globalvars.pr2, fulltraj_back, speed_factor=2.0)
    pr2_follow_traj(globalvars.pr2, fulltraj, speed_factor=4.0)
    time.sleep(2)

    #globalvars.or_sim.remove_objects([globalvars.cup])
    globalvars.pr2.update_rave()
else:
    print "in else statement"
    update_or_robot(robot, fulltraj[0][-1,:], rave_inds=fulltraj[1])
    globalvars.or_sim.viewer.Step()  # Updates OpenRave viewer, to show new robot configuration

# move_to_initial_position(globalvars.pr2, robot)
time.sleep(1)
