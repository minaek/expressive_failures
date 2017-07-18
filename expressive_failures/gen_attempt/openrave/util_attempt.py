import globalvars
from globalvars import *
import numpy as np, math
from math import *

"""Helper functions"""

# env = globalvars.env
# robot = globalvars.robot
# manip = globalvars.manip



def plotTraj(bodies,waypts, color=[0, 1, 0]):
    """
    Plots the best trajectory found or planned
    """
    global env,robot,manip
    for i in range(0,len(waypts)):
        dof = waypts[i]
        robot.SetDOFValues(dof, manip.GetArmIndices())
        coord = manip.GetEndEffectorTransform()[:3][:,3]     
        bodies.append(env.plot3(points=coord, pointsize=0.05, colors=(color[0],color[1],color[2]), drawstyle=1))


def transform2quat(transform):
    """Construct quaternion from the transform/rotation matrix 
    :returns: quaternion formed from transform matrix
    :rtype: numpy array
    """

    # Code was copied from perl PDL code that uses backwards index ordering
    T = transform.transpose()  
    den = np.array([ 1.0 + T[0,0] - T[1,1] - T[2,2],
                       1.0 - T[0,0] + T[1,1] - T[2,2],
                       1.0 - T[0,0] - T[1,1] + T[2,2],
                       1.0 + T[0,0] + T[1,1] + T[2,2]])
      
    max_idx = np.flatnonzero(den == max(den))[0]

    q = np.zeros(4)
    q[max_idx] = 0.5 * sqrt(max(den))
    denom = 4.0 * q[max_idx]
    if (max_idx == 0):
        q[1] =  (T[1,0] + T[0,1]) / denom 
        q[2] =  (T[2,0] + T[0,2]) / denom 
        q[3] = -(T[2,1] - T[1,2]) / denom 
    if (max_idx == 1):
        q[0] =  (T[1,0] + T[0,1]) / denom 
        q[2] =  (T[2,1] + T[1,2]) / denom 
        q[3] = -(T[0,2] - T[2,0]) / denom 
    if (max_idx == 2):
        q[0] =  (T[2,0] + T[0,2]) / denom 
        q[1] =  (T[2,1] + T[1,2]) / denom 
        q[3] = -(T[1,0] - T[0,1]) / denom 
    if (max_idx == 3):
        q[0] = -(T[2,1] - T[1,2]) / denom 
        q[1] = -(T[0,2] - T[2,0]) / denom 
        q[2] = -(T[1,0] - T[0,1]) / denom 

    return q

def get_euler(trans_matrix):
	""" 
	Get yaw, pitch, roll from transform matrix
	"""
	rot_matrix = trans_matrix[:3][:,:3]
	yaw = np.arctan(rot_matrix[0][1]/rot_matrix[0][0])

	pitch = np.arctan(-rot_matrix[0][2]/np.sqrt(rot_matrix[2][1]**2 + rot_matrix[2][2]**2))

	roll = np.arctan(rot_matrix[2][1]/rot_matrix[2][2])
	return yaw, pitch, roll


def ideal_traj(xyz_target):
    """ 
    Computes ideal trajectory to goal target position 
    ---
    input: goal coordinate position in global frame, output: trajectory

    """
    global ee_link_name,manip_name,nsteps, ee_link_z

    traj =  plan_follow_trajs(robot,manip_name,xyz_target, "I")

    robot.SetDOFValues(traj[-1], manip.GetArmIndices())
    ee_transform = manip.GetEndEffectorTransform()

    print " "
    print "current pos: " + str(ee_transform[:3][:,3])
    print "goal pos: " + str(ee_link_z) 
    print "starting pos: " + str(starting_transform[:3][:,3])

    return traj

