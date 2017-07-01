import roslib
roslib.load_manifest("control_msgs")
import rospy
import trajoptpy

from catkin.find_in_workspaces import find_in_workspaces
import commands, logging, math, numpy as np, os, time
import openravepy

from parse_csv import row_to_column
from lfd.rapprentice import PR2
from lfd.environment.simulation import DynamicSimulationRobotWorld
from lfd.environment.simulation_object import XmlSimulationObject, \
        TableSimulationObject, BoxSimulationObject
from lfd.demonstration import demonstration
from expressive_failures.util.pr2_util import open_gripper, close_gripper, \
                                              move_gripper, follow_body_traj
from expressive_failures.util.util import get_transform, rotation_z

# NOTE: before running any functions in this script, run the following commands:
#     roslaunch pr2capabilities_gazebo pr2capabilities.launch
#     roslaunch pr2capabilities_control pr2capabilities_traj_control.launch
#     rosrun gazebo_ros spawn_model -database plastic_cup_2 -sdf -model plastic_cup -x 0.66 -y 0 -z 0.48
# The first command initializes Gazebo, the second initializes the robot
# controllers in Gazebo, and the third places the cup on the table.
# The height of the cup (z) should be set to TABLE_HEIGHT + 0.03.

MODELS_DIR = os.path.expanduser("~/.gazebo/models")
CUP_MESH = "plastic_cup_2/meshes/plastic_cup_1-5_0-8x0-8y1-3z.dae"
TABLE_HEIGHT = 0.45

def initialize():
    openrave_sim = DynamicSimulationRobotWorld()
    robot_xml = XmlSimulationObject("robots/pr2-beta-static.zae", dynamic=False)
    openrave_sim.add_objects([robot_xml], consider_finger_collisions=True)
    table = BoxSimulationObject('table', \
                                [0.66, 0, (TABLE_HEIGHT + 0.03) / 2.0], \
                                [0.4, 0.75, (TABLE_HEIGHT + 0.03) / 2.0], \
                                dynamic=False)
    openrave_sim.add_objects([table], consider_finger_collisions=True)
    #table_xml = TableSimulationObject("table", \
    #                                  [0.66, 0, (TABLE_HEIGHT + 0.03) / 2.0],
    #                                  [0.4, 0.75, (TABLE_HEIGHT + 0.03) / 2.0],
    #                                  dynamic=False)
    #openrave_sim.add_objects([table_xml], consider_finger_collisions=True)
    cup = XmlSimulationObject(os.path.join(MODELS_DIR, CUP_MESH), dynamic=False)
    openrave_sim.add_objects([cup], consider_finger_collisions=True)

    v = trajoptpy.GetViewer(openrave_sim.env)  # TODO: not sure if this is necessary

    # Initialize interface for passing controls to Gazebo. Make sure to pass
    # in the OpenRave environment we just created, so PR2 isn't referring (and
    # updating) a separate OpenRave environment.
    pr2 = PR2.PR2(openrave_sim.env)
    time.sleep(0.2)

    return openrave_sim, pr2, v, cup
    
def move_cup(x, y, z, object_id='plastic_cup'):
    #    "'{model_state: { model_name: plastic_cup, pose: { position: " + \
    #    "{ x: " + str(x) + ", y: " + str(y) + ", z: " + str(z) + \
    #    "}, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: " + \
    #    "{x: 0.0 , y: 0 ,z: 0 }, angular: { x: 0.0 , y: 0 , z: 0.0 } }, " + \
    #    "reference_frame: world } }'")
    model_state = "'{model_state: { model_name: " + object_id + ", pose: " +  \
                  "{ position: { x: " + str(x) + ", y: " + str(y) + ", z: " + str(z) + \
                  "}, orientation: {x: 0, y: 0, z: 0, w: 0 } }, " + \
                  "twist: { linear: {x: 0.0 , y: 0 ,z: 0 }, " + \
                  "angular: { x: 0.0 , y: 0 , z: 0.0 } }, reference_frame: world } }'"
    o = commands.getstatusoutput("rosservice call /gazebo/set_model_state " + model_state)
    if o[1].split('\n')[0].split(':')[1].strip() != 'True':
        # moving cup failed
        print "ERROR: failed to move cup to new position"

def move_to_initial_position(pr2):
    # Move PR2 to initial position
    #pr2.torso.set_height(0.3)
    pr2.head.set_pan_tilt(0, 1.05)
    pr2.rarm.goto_posture('side')
    pr2.larm.goto_posture('side')
    open_gripper(pr2, 'r')
    open_gripper(pr2, 'l')
    pr2.join_all()
    time.sleep(3)
    pr2.update_rave()

def pick_up_cup(or_sim, pr2, cup, cup_x=0.66, cup_y=0, cup_z=TABLE_HEIGHT+0.03):
    lr = 'r'  # Execute task with just right gripper
    cup.set_state([get_transform(cup_x, cup_y, cup_z + 0.09)])
    cup.get_bullet_objects()[0].UpdateRave()

    D = 0.035  # distance from gripper point (fingertips when closed) to center
    R = D * 0.67  # cup radius
    PICK_UP_HEIGHT = 0.07 + 0.04
    ADJ_D = D

    move_to_initial_position(pr2)
    move_cup(cup_x+0.005, cup_y, cup_z)

    # Move gripper to cup and grasp it
    gripper_end = np.r_[R + ADJ_D*math.cos(np.pi/4), \
                        R + ADJ_D*math.sin(np.pi/4), \
                        cup_z+PICK_UP_HEIGHT]
    full_traj_tocup, pose_costs = \
            move_gripper(or_sim, gripper_end, R_end=rotation_z(np.pi/4), lr=lr, \
                         beta_rot=100000.0, n_steps=100, grasp_cup=True, R=R, \
                         D=D, cup_xyz=np.r_[cup_x, cup_y, cup_z+PICK_UP_HEIGHT])
    aug_traj_tocup = \
            demonstration.AugmentedTrajectory.create_from_full_traj(pr2.robot, full_traj_tocup)
    bodypart2traj = {}
    for lr, arm_traj in aug_traj_tocup.lr2arm_traj.items():
        part_name = {"l":"larm", "r":"rarm"}[lr]
        bodypart2traj[part_name] = arm_traj
    follow_body_traj(pr2, bodypart2traj, speed_factor=0.2)
    close_gripper(pr2, lr)
    time.sleep(2)
    or_sim.remove_objects([cup])
    pr2.update_rave()

    # Lift up cup
    gripper_end = np.r_[0.66, 0, TABLE_HEIGHT+0.03+0.5]
    full_traj_pickup, _ = move_gripper(or_sim, gripper_end, R_end=rotation_z(np.pi/2), \
                                       lr=lr, n_steps=30)
    aug_traj_pickup = \
            demonstration.AugmentedTrajectory.create_from_full_traj(pr2.robot, full_traj_pickup)
    bodypart2traj = {}
    for lr, arm_traj in aug_traj_pickup.lr2arm_traj.items():
        part_name = {"l":"larm", "r":"rarm"}[lr]
        bodypart2traj[part_name] = arm_traj
    follow_body_traj(pr2, bodypart2traj, speed_factor=0.5)
    time.sleep(2)
    or_sim.add_objects([cup], consider_finger_collisions=True)
    pr2.update_rave()

    o = commands.getstatusoutput("rosservice call gazebo/get_model_state '{model_name: plastic_cup}'")
    cup_z_after = float(o[1].split('\n')[4].split(':')[1])
    return cup_z_after > TABLE_HEIGHT + 0.2

def main():
    rospy.init_node("manip_task",disable_signals=True)
    or_sim, pr2, v, cup = initialize()
    success = pick_up_cup(or_sim, pr2, cup)
    print "Success:", success

if __name__ == "__main__":
    main()
