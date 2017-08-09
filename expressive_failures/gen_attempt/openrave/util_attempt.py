import globalvars
from globalvars import *
import numpy as np, math
from math import *

"""Helper functions"""

# env = globalvars.env
# robot = globalvars.robot
# manip = globalvars.manip


def get_rot_deltas(waypt, link_name, starting_config, goal_config):
    global robot, manip
    armids = list(manip.GetArmIndices()) #get arm indices
    links = robot.GetLinks()
    link_idx = links.index(robot.GetLink(link_name))

    #current
    robot.SetDOFValues(waypt,armids) # set dof values to current config
    EEcurr_x = manip.GetEndEffectorTransform()[:3][:,0] #get EE q
    EEcurr_y = manip.GetEndEffectorTransform()[:3][:,1] #get EE q
    EEcurr_z = manip.GetEndEffectorTransform()[:3][:,2] #get EE q

    linkstrans = robot.GetLinkTransformations()
    Lcurr_x = linkstrans[link_idx][:3][:,0] #get xyz of specific link
    Lcurr_y = linkstrans[link_idx][:3][:,1] #get xyz of specific link
    Lcurr_z = linkstrans[link_idx][:3][:,2] #get xyz of specific link

    #ideal
    robot.SetDOFValues(goal_config,armids) #set dof valuue to ideal config
    EEdesired_x = manip.GetEndEffectorTransform()[:3][:,0] #get EE q
    EEdesired_y = manip.GetEndEffectorTransform()[:3][:,1] #get EE q
    EEdesired_z = manip.GetEndEffectorTransform()[:3][:,2] #get EE q  
    
    linkstrans_d = robot.GetLinkTransformations()
    Ldesired_x = linkstrans_d[link_idx][:3][:,0]
    Ldesired_y = linkstrans_d[link_idx][:3][:,1]
    Ldesired_z = linkstrans_d[link_idx][:3][:,2]
    
    #start
    robot.SetDOFValues(starting_config,armids) #set dof valuue to starting config
    EEstart_x = manip.GetEndEffectorTransform()[:3][:,0] #get EE q
    EEstart_y = manip.GetEndEffectorTransform()[:3][:,1] #get EE q
    EEstart_z = manip.GetEndEffectorTransform()[:3][:,2] #get EE q  

    linkstrans_s = robot.GetLinkTransformations()
    Lstart_x = linkstrans_s[link_idx][:3][:,0]
    Lstart_y = linkstrans_s[link_idx][:3][:,1]
    Lstart_z = linkstrans_s[link_idx][:3][:,2]


    deltaEE_x = (EEdesired_x-EEstart_x)
    deltaEE_y = (EEdesired_y-EEstart_y)
    deltaEE_z = (EEdesired_z-EEstart_z)

    deltaL_x = (Lcurr_x-Lstart_x)
    deltaL_y = (Lcurr_y-Lstart_y)
    deltaL_z = (Lcurr_z-Lstart_z)

    return deltaEE_x, deltaEE_y, deltaEE_z, deltaL_x, deltaL_y, deltaL_z

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])
def c():
    """
    Plots the best trajectory found or planned
    """
    global env,robot,manip
armids = list(manip.GetArmIndices()) #get arm indices
links = robot.GetLinks()
link_idx = links.index(robot.GetLink('r_shoulder_lift_link'))
linkstrans = robot.GetLinkTransformations()
Lcurr = linkstrans[link_idx][:3][:,3] #get xyz of specific link
env.plot3(points=Lcurr, pointsize=0.05, colors=(1,1,0), drawstyle=1)


def plotTraj(bodies,waypts, color=[1, 1, 0]):
    """
    Plots the best trajectory found or planned
    """
    global env,robot,manip
    for i in range(0,len(waypts)):
        dof = waypts[i]
        robot.SetDOFValues(dof, manip.GetArmIndices())
        coord = manip.GetEndEffectorTransform()[:3][:,3]     
        bodies.append(env.plot3(points=coord, pointsize=0.05, colors=(1,1,0), drawstyle=1))


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



