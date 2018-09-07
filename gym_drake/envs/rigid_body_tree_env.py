import gym
import numpy as np
from pydrake.all import RigidBodyTree, RigidBodyPlant
from drake_env import DrakeEnv

class RigidBodyTreeEnv(DrakeEnv):
    '''
    Implements a DrakeEnv for models specified by a RigidBodyTree. Constructs
    the RigidBodyPlant for simulation and DrakeVisualizer for visualization.
    '''

    def __init__(self, tree):
        self.tree = tree
        RigidBodyTree.__init__(self)

    def get_plant_system(self):
        '''
        Implements the get_mdp_diagram method in DrakeEnv by constructing a RigidBodyPlant
        '''
        return builder.AddSystem(RigidBodyPlant(tree))

    @property action_space
    def action_space(self):
        return spaces.Box(*self.action_limits)

    @property observation_space
    def action_space(self):
        return spaces.Box(*self.observation_limits)

    @property
    def action_limits(self):
        '''
        Subclasses should implement their own action limits
        '''
        raise NotImplementedError

    @property
    def observation_limits(self):
        '''
        Subclasses should implement their own observation limits
        '''
        raise NotImplementedError


    def get_reward(self, state, action):
        '''
        Subclasses should implement their own reward functions 
        '''
        raise NotImplementedError

    def init_visualizer(self):
        return DrakeVisualizer(tree_)
