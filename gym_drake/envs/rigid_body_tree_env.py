import gym
import numpy as np
from pydrake.all import RigidBodyTree, RigidBodyPlant
from drake_env import DrakeEnv

class RigidBodyTreeEnv(DrakeEnv):
    '''
    Implements a DrakeEnv for models specified by a RigidBodyTree. Constructs
    the RigidBodyPlant for simulation and DrakeVisualizer for visualization.
    '''

    def __init__(self, tree, limits):
        dt = 0.01
        self._action_limits = limits['action']
        self._observation_limits = limits['observation']
        super(DrakeEnv, self).__init__()

    @property
    def action_limits(self):
        return self._action_limits

    @property
    def observation_limits(self):
        return self._observation_limits

    def get_mdp_diagram(self):
        '''
        Constructs the Pendulum MDP with a quadratic reward that is maximized at np.pi
        '''

    def init_visualizer(self):
        return DrakeVisualizer(tree_)
