from __future__ import division

import numpy as np
from lfd.rapprentice import resampling, math_utils as mu
from lfd.environment import sim_util # TODO fold in sim_util function into LfdEnvironment

class AugmentedTrajectory(object):
    def __init__(self, lr2arm_traj=None, lr2finger_traj=None, lr2ee_traj=None, lr2open_finger_traj=None, lr2close_finger_traj=None):
        """Inits AugmentedTrajectory
        
        Args:
            lr2arm_traj: dict that maps from 'l' and/or 'r' to the left arm's and/or right arm's joint angle trajectory
            lr2finger_traj: dict that maps to the left gripper's and/or right gripper's finger joint angle trajectory
            lr2ee_traj: dict that maps to the left gripper's and/or right gripper's end-effector trajectory (i.e. a numpy.array of homogeneous matrices)
            lr2open_finger_traj: dict that maps to a boolean vector indicating whether there is a opening action for the left gripper and/or right gripper for every time step. By default, there is no action for all time steps.
            lr2close_finger_traj: same as lr2open_finger_traj but for closing action
        """
        # make sure all trajs have the same number of steps
        self.n_steps = None
        for lr2traj in [lr2arm_traj, lr2finger_traj, lr2ee_traj, lr2open_finger_traj, lr2close_finger_traj]:
            if lr2traj is None:
                continue
            for lr in lr2traj.keys():
                if self.n_steps is None:
                    self.n_steps = lr2traj[lr].shape[0]
                else:
                    assert lr2traj[lr].shape[0] == self.n_steps

        if lr2arm_traj is None:
            self.lr2arm_traj = {}
        else:
            self.lr2arm_traj = lr2arm_traj
        
        if lr2finger_traj is None:
            self.lr2finger_traj = {}
        else:
            self.lr2finger_traj = lr2finger_traj
        
        if lr2ee_traj is None:
            self.lr2ee_traj = {}
        else:
            self.lr2ee_traj = lr2ee_traj
        
        if lr2open_finger_traj is None:
            self.lr2open_finger_traj = {}
            for lr in 'lr':
                self.lr2open_finger_traj[lr] = np.zeros(self.n_steps, dtype=bool)
        else:
            self.lr2open_finger_traj = lr2open_finger_traj
        if lr2close_finger_traj is None:
            self.lr2close_finger_traj = {}
            for lr in 'lr':
                self.lr2close_finger_traj[lr] = np.zeros(self.n_steps, dtype=bool)
        else:
            self.lr2close_finger_traj = lr2close_finger_traj
    
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            for (lr2traj, other_lr2traj) in [(self.lr2arm_traj, other.lr2arm_traj), (self.lr2finger_traj, other.lr2finger_traj),
                                             (self.lr2ee_traj, other.lr2ee_traj),
                                             (self.lr2open_finger_traj, other.lr2open_finger_traj), (self.lr2close_finger_traj, other.lr2close_finger_traj)]:
                if lr2traj is None:
                    if other_lr2traj is None:
                        continue
                    else:
                        return False
                if set(lr2traj.keys()) != set(other_lr2traj.keys()):
                    return False
                for lr in lr2traj.keys():
                    if np.any(lr2traj[lr] != other_lr2traj[lr]):
                        return False
            return True
        else:
            return False
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    @staticmethod
    def create_from_full_traj(robot, full_traj, lr2open_finger_traj=None, lr2close_finger_traj=None):
        traj, dof_inds = full_traj
        lr2arm_traj = {}
        for lr in 'lr':
            manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
            arm_inds = robot.GetManipulator(manip_name).GetArmIndices()
            if set(arm_inds).intersection(set(dof_inds)):
                if not set(arm_inds).issubset(set(dof_inds)):
                    raise RuntimeError("Cannot create AugmentedTrajectory from incomplete full_traj")
                arm_traj = np.zeros((traj.shape[0], len(arm_inds)))
                for (j,arm_ind) in enumerate(arm_inds):
                    arm_traj[:,j] = traj[:,dof_inds.index(arm_ind)]
                lr2arm_traj[lr] = arm_traj
        lr2finger_traj = {}
        for lr in 'lr':
            finger_ind = robot.GetJointIndex("%s_gripper_l_finger_joint"%lr)
            if finger_ind in dof_inds:
                lr2finger_traj[lr] = traj[:,dof_inds.index(finger_ind)][:,None]
        lr2ee_traj = {}    
        for lr in lr2arm_traj.keys():
            lr2ee_traj[lr] = sim_util.get_ee_traj(robot, lr, lr2arm_traj[lr])
        return AugmentedTrajectory(lr2arm_traj=lr2arm_traj, lr2finger_traj=lr2finger_traj, lr2ee_traj=lr2ee_traj, lr2open_finger_traj=lr2open_finger_traj, lr2close_finger_traj=lr2close_finger_traj)
    
    def get_full_traj(self, robot):
        """
        TODO: remove sim_util.get_full_traj
        """
        trajs = []
        dof_inds = []
        for lr in 'lr':
            if lr in self.lr2arm_traj:
                arm_traj = self.lr2arm_traj[lr]
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                trajs.append(arm_traj)
                dof_inds.extend(robot.GetManipulator(manip_name).GetArmIndices())
        for lr in 'lr':
            if lr in self.lr2finger_traj:
                finger_traj = self.lr2finger_traj[lr]
                trajs.append(finger_traj)
                dof_inds.append(robot.GetJointIndex("%s_gripper_l_finger_joint"%lr))
        if len(trajs) > 0:
            full_traj = (np.concatenate(trajs, axis=1), dof_inds)
        else:
            full_traj = (np.zeros((0,0)), [])
        return full_traj
    
    def get_resampled_traj(self, timesteps_rs):
        lr2arm_traj_rs = None if self.lr2arm_traj is None else {}
        lr2finger_traj_rs = None if self.lr2finger_traj is None else {}
        for (lr2traj_rs, self_lr2traj) in [(lr2arm_traj_rs, self.lr2arm_traj), (lr2finger_traj_rs, self.lr2finger_traj)]:
            if self_lr2traj is None:
                continue
            for lr in self_lr2traj.keys():
                lr2traj_rs[lr] = mu.interp2d(timesteps_rs, np.arange(len(self_lr2traj[lr])), self_lr2traj[lr])

        if self.lr2ee_traj is None:
            lr2ee_traj_rs = None
        else:
            lr2ee_traj_rs = {}
            for lr in self.lr2ee_traj.keys():
                lr2ee_traj_rs[lr] = np.asarray(resampling.interp_hmats(timesteps_rs, np.arange(len(self.lr2ee_traj[lr])), self.lr2ee_traj[lr]))
        
        lr2open_finger_traj_rs = None if self.lr2open_finger_traj is None else {}
        lr2close_finger_traj_rs = None if self.lr2close_finger_traj is None else {}
        for (lr2oc_finger_traj_rs, self_lr2oc_finger_traj) in [(lr2open_finger_traj_rs, self.lr2open_finger_traj), (lr2close_finger_traj_rs, self.lr2close_finger_traj)]:
            if self_lr2oc_finger_traj is None:
                continue
            for lr in self_lr2oc_finger_traj.keys():
                self_oc_finger_traj = self_lr2oc_finger_traj[lr]
                self_oc_inds = np.where(self_oc_finger_traj)[0]
                oc_inds_rs = np.abs(timesteps_rs[:,None] - self_oc_inds[None,:]).argmin(axis=0)
                oc_finger_traj_rs = np.zeros(len(timesteps_rs), dtype=bool)
                oc_finger_traj_rs[oc_inds_rs] = True
                lr2oc_finger_traj_rs[lr] = oc_finger_traj_rs
        
        return AugmentedTrajectory(lr2arm_traj=lr2arm_traj_rs, lr2finger_traj=lr2finger_traj_rs, lr2ee_traj=lr2ee_traj_rs, lr2open_finger_traj=lr2open_finger_traj_rs, lr2close_finger_traj=lr2close_finger_traj_rs)
    
    def __repr__(self):
        return "%s(...)" % (self.__class__.__name__)
