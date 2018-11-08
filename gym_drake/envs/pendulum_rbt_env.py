import numpy as np
from gym_drake.envs import rigid_body_tree_env
from pydrake.all import FloatingBaseType
from utils import wrap_angle

class PendulumRBTEnv(rigid_body_tree_env.RigidBodyTreeEnv):
    use_shaped_reward = True
    reset_random = True

    def __init__(self):
        # Call super-class constructor with the model path
        super(PendulumRBTEnv, self).__init__("Pendulum.urdf", FloatingBaseType.kFixed)
        self.state_des = np.array([np.pi, 0])
        self.Q = np.diag([1,.1])
        self.R = np.diag([0.001])
        self.eps = 1e-2

    @property
    def action_limits(self):
        return (np.array([-3.0]), np.array([3.0]))

    @property
    def observation_limits(self):
        return (np.array([-1, -1, -np.inf]),
                np.array([1, 1, np.inf]))

    def get_observation(self):
        s = self.get_state()
        ob = np.array([np.sin(s[0]), np.cos(s[0]), s[1]])
        return ob

    @property
    def dt(self):
        return 0.01

    def get_reward(self, state, action):
        err = self.state_des - state
        err[0] = wrap_angle(err[0])
        if self.use_shaped_reward:
            # quadratic cost on the error and action
            return -err.dot(self.Q).dot(err) - action.dot(self.R).dot(action)
        else:
            # sparse reward
            return 1.0 if err < eps else 0.0

    def is_done(self):
        '''
        Returns true if the system is close enough to the goal
        '''
        err = self.state_des - self.get_state()
        err[0] = wrap_angle(err[0])
        return err.dot(err) < self.eps or self.context.get_time() > 20

    def reset_state(self):
        # random state set uniformly between (-pi, -1) (+pi, +1)
        if self.reset_random:
            new_state = (np.random.random((2))-.5)*np.array([2*np.pi,2])
        else:
            new_state = np.zeros(2)
        self.set_state(new_state)