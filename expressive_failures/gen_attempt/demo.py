import pr2_util
import roslib
roslib.load_manifest("control_msgs")
import rospy
import trajoptpy
from catkin.find_in_workspaces import find_in_workspaces
import commands, logging, math, numpy as np, os, time
import openravepy
from lfd.rapprentice import PR2
from lfd.environment.simulation import DynamicSimulationRobotWorld
from lfd.environment.simulation_object import XmlSimulationObject, \
        TableSimulationObject, BoxSimulationObject
from lfd.demonstration import demonstration
from lfd.util import util


# NOTE: before running any functions in this script, run the following commands:
#     roslaunch pr2capabilities_gazebo pr2capabilities_gazebo.launch
#     roslaunch pr2capabilities_control pr2capabilities_traj_control.launch
#     rosrun gazebo_ros spawn_model -database plastic_cup_2 -sdf -model plastic_cup -x 0.66 -y 0 -z 0.48
# The first command initializes Gazebo, the second initializes the robot
# controllers in Gazebo, and the third places the cup on the table.
# The height of the cup (z) should be set to TABLE_HEIGHT + 0.03.

MODELS_DIR = os.path.expanduser("~/.gazebo/models")
CUP_MESH = "plastic_cup_2/meshes/plastic_cup_1-5_0-8x0-8y1-3z.dae"
TABLE_HEIGHT = 0.45
LOAD_GAZEBO = False

def initialize(load_gazebo=True):
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
    if load_gazebo:
        pr2 = PR2.PR2(env=openrave_sim.env)
        time.sleep(0.2)
    else:
        pr2 = None

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

def place_cup(x, y, z, object_id='plastic_cup'):
    subprocess.Popen(['rosrun', 'gazebo_ros', 'spawn_model', '-database',
                      'plastic_cup_2', '-sdf', '-model', object_id,
                      '-x', str(x), '-y', str(y), '-z', str(z)])

def remove_cup(model_id='plastic_cup'):
    subprocess.Popen(['rosservice', 'call', 'gazebo/delete_model', "'{model_name: " + model_id + "}'"])

def update_or_robot(robot, joint_vals, ros_to_rave=False, rave_inds=None):
    if ros_to_rave:
        ros_values = joint_vals
        rave_values = [ros_values[i_ros] for i_ros in PR2.GOOD_ROS_INDS]
        robot.SetJointValues(rave_values[:20], PR2.ALL_RAVE_INDS[:20])
        robot.SetJointValues(rave_values[20:], PR2.ALL_RAVE_INDS[20:])   
    elif rave_inds is not None:
        robot.SetJointValues(joint_vals, rave_inds)
    else:
        robot.SetJointValues(joint_vals)

def move_to_initial_position(pr2, or_pr2):
    # Move PR2 to initial position
    if pr2 is not None:
        #pr2.torso.set_height(0.3)
        pr2.head.set_pan_tilt(0, 1.05)
        pr2.rarm.goto_posture('side')
        pr2.larm.goto_posture('side')
        pr2_util.open_gripper(pr2, 'r')
        pr2_util.open_gripper(pr2, 'l')
        pr2.join_all()
        time.sleep(3)
        pr2.update_rave()
    else:
        init_joint_vals = [-9.548349400034795e-07, 0.007108477115980172, \
                0.008535662439170366, 9.108906606769551e-07, 0.01342759549560313, \
                0.015541732430619069, 2.3592229583258018e-06, -0.007190925151169481, \
                -0.006547917447990771, -2.440460509234299e-06, -0.007291211060291403, \
                -0.006701316959966341, 0.011497875265269247, 0.0, 1.3353169938667975e-06, \
                1.1014890829727761, -0.14584224062866813, -1.010999999980605, \
                -1.8320000000141485, -0.3320000000114023, -1.0999999999921979, \
                -1.4370000000050762, -2.0000006405930977, -3.073999999972446, \
                0.08000002637629851, 0.46313299645051387, 0.46313299645051387, \
                0.46313299645051387, 0.46313299645051387, 0.0, 0.0, \
                1.0109999999819799, 1.8320000000517975, -0.33199999997730156, \
                1.10000000001222, -1.4369999999995882, -2.0000006542547393, \
                3.0740000000266487, 0.08000005416076225, 0.4631331720114844, \
                0.4631331720114844, 0.4631331720114844, 0.4631331720114844, 0.0, 0.0]
        update_or_robot(or_pr2, init_joint_vals, ros_to_rave=True)


def grasp_cup(or_sim, pr2, cup, cup_x=0.66, cup_y=0, cup_z=TABLE_HEIGHT+0.03):
    or_pr2 = or_sim.env.GetRobots()[0]

    lr = 'r'  # Execute task with just right gripper
    cup.set_state([util.get_transform(cup_x, cup_y, cup_z + 0.09)])
    cup.get_bullet_objects()[0].UpdateRave()

    D = 0.035  # distance from gripper point (fingertips when closed) to center
    R = D * 0.67  # cup radius
    PICK_UP_HEIGHT = 0.11  # 0.11
    ADJ_D = D

    move_to_initial_position(pr2, or_pr2)
    move_cup(cup_x+0.005, cup_y, cup_z)

    # Move gripper to cup and grasp it
    gripper_end = np.r_[R + ADJ_D*math.cos(np.pi/4), \
                        R + ADJ_D*math.sin(np.pi/4), \
                        cup_z+PICK_UP_HEIGHT]
    full_traj_tocup, pose_costs = \
            pr2_util.move_gripper(or_sim, gripper_end, R_end=util.rotation_z(np.pi/4), lr=lr, \
                         beta_rot=100000.0, n_steps=100, grasp_cup=True, R=R, \
                         D=D, cup_xyz=np.r_[cup_x, cup_y, cup_z+PICK_UP_HEIGHT])

    if pr2 is not None:
        aug_traj_tocup = \
                demonstration.AugmentedTrajectory.create_from_full_traj(pr2.robot, full_traj_tocup)
        bodypart2traj = {}
        for lr, arm_traj in aug_traj_tocup.lr2arm_traj.items():
            part_name = {"l":"larm", "r":"rarm"}[lr]
            bodypart2traj[part_name] = arm_traj
        pr2_util.follow_body_traj(pr2, bodypart2traj, speed_factor=0.2)
        pr2_util.close_gripper(pr2, lr)
        time.sleep(2)

        or_sim.remove_objects([cup])
        pr2.update_rave()
    else:
        update_or_robot(or_pr2, full_traj_tocup[0][-1,:], rave_inds=full_traj_tocup[1])

    manip = or_pr2.GetActiveManipulator()
    starting_config = manip.GetArmDOFValues()
    return starting_config



# def main():
#     rospy.init_node("manip_task",disable_signals=True)
#     or_sim, pr2, v, cup = initialize(load_gazebo=LOAD_GAZEBO)
#     grasp_cup(or_sim, pr2, cup)
#     #success = grasp_cup(or_sim, pr2, cup)
#     #print "Success:", success

# if __name__ == "__main__":
    # rospy.init_node("manip_task",disable_signals=True)
    # or_sim, pr2, v, cup = initialize(load_gazebo=LOAD_GAZEBO)
    # starting_config = grasp_cup(or_sim, pr2, cup)
    # attempt_traj(or_sim,pr2,starting_config)
# if __name__ == '__main__':
#     print "generating attempt trajectory.."
#     attempt_traj(or_sim,pr2,starting_config)