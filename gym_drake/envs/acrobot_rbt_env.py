import numpy as np
from gym_drake.envs import rigid_body_tree_env
from pydrake.all import FloatingBaseType

class AcrobotRBTEnv(rigid_body_tree_env.RigidBodyTreeEnv):
    use_shaped_reward = True

    def __init__(self):
        # Call super-class constructor with the model path
        super(AcrobotRBTEnv, self).__init__("Acrobot.urdf", FloatingBaseType.kFixed)
        self.state_des = np.array([0, np.pi, 0, 0])
        self.eps = 1e-2

    @property
    def action_limits(self):
        return (np.array([-20.0]), np.array([20.0]))

    @property
    def observation_limits(self):
        return (np.array([-np.inf, -np.inf, -np.inf, -np.inf]),
                np.array([np.inf, np.inf, np.inf, np.inf]))

    @property
    def dt(self):
        return 0.05

    def get_reward(self, state, action):
        err = self.state_des - state
        if self.use_shaped_reward:
            # quadratic cost on the error and action
            return -err.dot(err)# - action.dot(action)
        else:
            # sparse reward
            return 1.0 if err < eps else 0.0

    def is_done(self):
        '''
        Returns true if the acrobot is close enough to the goal
        '''
        err = self.state_des - self.get_state()
        return err.dot(err) < self.eps or self.context.get_time() > 10
