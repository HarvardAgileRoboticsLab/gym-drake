import os
import numpy as np
from gym_drake.envs import rigid_body_tree_env
from pydrake.all import FloatingBaseType, getDrakePath

class KukaArmRBTEnv(rigid_body_tree_env.RigidBodyTreeEnv):
    use_shaped_reward = True
    reset_random = True

    def __init__(self):
        # Call super-class constructor with the model path
        model_path = os.path.join(getDrakePath(), "manipulation", "models",
                                     "iiwa_description", "urdf", "iiwa14_no_collision.urdf")
        super(KukaArmRBTEnv, self).__init__(model_path, FloatingBaseType.kFixed)
        self.state_des = np.zeros(14)
        self.Q = np.eye(14)
        self.R = 1e-3*np.eye(7)
        self.eps = 1e-2

    @property
    def action_limits(self):
        return (-300*np.pi*np.ones(7), 300*np.pi*np.ones(7))

    @property
    def observation_limits(self):
        pos_limits = np.pi*np.ones(7)
        vel_limits = 10*np.ones(7)
        return -np.hstack([pos_limits,vel_limits]), np.hstack([pos_limits,vel_limits])

    @property
    def dt(self):
        return 0.01

    def get_reward(self, state, action):
        err = self.state_des - state
        if self.use_shaped_reward:
            # quadratic cost on the error and action
            return -err.dot(self.Q).dot(err) - action.dot(self.R).dot(action)
        else:
            # sparse reward
            return 1.0 if err.dot(err) < eps else 0.0

    def is_done(self):
        '''
        Returns true if the system is close enough to the goal
        '''
        err = self.state_des - self.get_state()
        return err.dot(err) < self.eps or self.context.get_time() > 20

    def reset_state(self):
        # random state set uniformly between +/- 3/4*pi
        if self.reset_random:
            new_pos = (np.random.random((7))-.5)*1.5*np.pi
            new_vel = np.zeros(7)
            new_state = np.hstack([new_pos, new_vel])
        else:
            new_state = np.zeros(7)
        self.set_state(new_state)