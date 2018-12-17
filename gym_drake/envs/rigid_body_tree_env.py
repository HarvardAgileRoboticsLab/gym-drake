import gym
import numpy as np
from pydrake.all import RigidBodyTree, RigidBodyPlant, AddModelInstanceFromUrdfFile, FloatingBaseType
from gym_drake.envs import drake_env
from gym_drake.envs import meshcat_visualizer_rbt
from .utils import get_full_model_path

class RigidBodyTreeEnv(drake_env.DrakeEnv):
    '''
    Implements a DrakeEnv for models specified by a RigidBodyTree. Constructs
    the RigidBodyPlant for simulation and DrakeVisualizer for visualization.
    '''

    def __init__(self, model_path, floating_base_type):
        self.model_path = get_full_model_path(model_path)
        self.tree = RigidBodyTree(self.model_path, floating_base_type)
        self._visualizer = None
        super(RigidBodyTreeEnv, self).__init__()

    @property
    def visualizer(self):
        if self._visualizer is None:
            self._visualizer = meshcat_visualizer_rbt.MeshcatVisualizerRBT(self.tree)
        return self._visualizer

    def plant_system(self):
        '''
        Implements the plant_system method in DrakeEnv by constructing a RigidBodyPlant
        '''
        self.rbp = RigidBodyPlant(self.tree)
        return self.rbp

    def get_input_port_action(self):
        '''
        Returns the system input port that corresponds to the action
        '''
        return self.rbp.actuator_command_input_port()

    def get_observation(self):
        return self.get_state()
    
    def get_state(self):
        '''
        Returns the system output port that corresponds to the observation
        '''
        rbp_context = self.simulator.get_context()
        return rbp_context.get_continuous_state().get_vector().get_value()

    def set_state(self, state):
        '''
        Sets the system state
        '''
        rbp_context = self.simulator.get_context()
        self.rbp.set_state_vector(rbp_context, state)

    @property
    def action_space(self):
        return gym.spaces.Box(*self.action_limits, dtype=np.float32)

    @property
    def observation_space(self):
        return gym.spaces.Box(*self.observation_limits, dtype=np.float32)

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

    def render(self, mode='human', close=False):
        '''
        Notifies the visualizer to redraw the image.
        '''
        self.visualizer.draw(self.get_state())