import gym
import numpy as np
from pydrake.all import RigidBodyTree, RigidBodyPlant
from gym_drake.envs import drake_env
from meshcat_rigid_body_visualizer import MeshcatRigidBodyVisualizer

class RigidBodyTreeEnv(drake_env.DrakeEnv):
    '''
    Implements a DrakeEnv for models specified by a RigidBodyTree. Constructs
    the RigidBodyPlant for simulation and DrakeVisualizer for visualization.
    '''

    def __init__(self, tree):
        self.tree = tree
        self._visualizer = None
        RigidBodyTree.__init__(self)

    def get_plant_system(self):
        '''
        Implements the get_mdp_diagram method in DrakeEnv by constructing a RigidBodyPlant
        '''
        return builder.AddSystem(RigidBodyPlant(tree))

    @property
    def visualizer(self):
        if self._visualizer is None:
            self._visualizer = MeshcatRigidBodyVisualizer(self.tree, draw_collision=True)
        return self._visualizer

    @property
    def action_space(self):
        return spaces.Box(*self.action_limits)

    @property
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
        return MeshcatRigidBodyVisualizer(tree_)
