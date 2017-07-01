import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()

import openravepy
from openravepy import *
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku
import time 


env = openravepy.Environment()
env.SetViewer('qtcoin')

env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
env.Load("../data/table.xml")
viewer = env.GetViewer()
viewer.SetCamera(
  np.array([[ -9.94542311e-03,   5.43414497e-01,  -8.39405607e-01,
          2.32253075e+00],
       [  9.99869989e-01,   1.60593593e-02,  -1.45012309e-03,
          3.47638549e-03],
       [  1.26922983e-02,  -8.39310897e-01,  -5.43503564e-01,
          1.97211254e+00],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]]))

#trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting

robot = env.GetRobots()[0]
joint_start = [0,0,0,0,0,0,0]
robot.SetDOFValues(joint_start, robot.GetManipulator('rightarm').GetArmIndices())


#reaching goal pose: [8.54594490e-01, -1.88000000e-01,  1.19776467e+00]


#def traj(xyz_target):
#goal is an xyz coordinate in the global frame

xyz_target = [8.54594490e-01, -1.88000000e-01,  1.19776467e+00]

#calculate coordinate with largest distance
manip = robot.GetManipulator("rightarm")
xyz_init = manip.GetTransform()[:,3][:3] #3x1
difference = (xyz_init-xyz_target)
coord = np.argmax(np.abs(difference)) #0=x,1=y or 2=z difference[coord] would give u largest diff
sign  = np.sign(difference[coord])

#now move to goal position such that the robot's joints all move in the direction of the coorindate
num_waypoints = 4
quat_target = [1,0,0,0] # wxyz
hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

# BEGIN ik
robot.SetActiveManipulator(manip)
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
    robot, iktype=openravepy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()
init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_palm_link",
    filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)

# END ik

request = {
  "basic_info" : {
    "n_steps" : 10,
    "manip" : "rightarm", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
  },
  {
    "type" : "collision",
    "name" :"cont_coll", # shorten name so printed table will be prettier
    "params" : {
      "continuous" : True,
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
    }
  }
  ],
  "constraints" : [
  # BEGIN pose_constraint
  {
    "type" : "pose", 
    "params" : {"xyz" : xyz_target, 
                "wxyz" : quat_target, 
                "link": "r_gripper_tool_frame",
                "timestep" : 9
                }              
  }
  # END pose_constraint
  ],
  # BEGIN init
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : init_joint_target.tolist() # need to convert numpy array to list
  }
  # END init
}

if args.position_only: request["constraints"][0]["params"]["rot_coeffs"] = [0,0,0]

s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem

# for t in range(1,num_waypts_plan): 
#   prob.AddCost(table_cost, [(t,j) for j in range(7)], "table%i"%t)



################################################################################

result = trajoptpy.OptimizeProblem(prob) # do optimization
print result

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

# Now we'll check to see that the final constraint was satisfied
robot.SetActiveDOFValues(result.GetTraj()[-1])
posevec = openravepy.poseFromMatrix(robot.GetLink("r_gripper_tool_frame").GetTransform())
quat, xyz = posevec[0:4], posevec[4:7]

# quat *= np.sign(quat.dot(quat_target))
# if args.position_only:
#     assert (quat - quat_target).max() > 1e-3
# else:
#     assert (quat - quat_target).max() < 1e-3

time.sleep(50)