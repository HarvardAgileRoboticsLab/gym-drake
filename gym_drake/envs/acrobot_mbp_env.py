import numpy as np
import pdb; pdb.set_trace()
from gym_drake.envs import multi_body_plant_env


class AcrobotMBPEnv(multi_body_plant_env.MultiBodyPlantEnv):
    def __init__(self):
        # Call super-class constructor with the model path
        super(AcrobotMBPEnv, self).__init__("models/acrobot.sdf")

    @property
    def action_limits(self):
        return (np.array([-2.0]), np.array([2.0]))

    @property
    def observation_limits(self):
        return (np.array([-np.inf, -np.inf]),
                        np.array([np.inf, np.inf]))

    @property
    def dt(self):
        return 0.1

    def get_reward(self, state, action):
        return 1 # TODO: Implement this
