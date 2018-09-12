import gym
import numpy as np
from pydrake.all import (
    AddModelFromSdfFile,
    DiagramBuilder,
    MultibodyPlant,
    SceneGraph,
    UniformGravityFieldElement,
)
from gym_drake.envs import drake_env
from meshcat_visualizer_mbp import MeshcatVisualizerMBP


class MultiBodyTreeEnv(drake_env.DrakeEnv):
    '''
    Implements a DrakeEnv for models specified by a RigidBodyTree. Constructs
    the RigidBodyPlant for simulation and DrakeVisualizer for visualization.
    '''

    def __init__(self, fname):
        self.fname = fname
        self._visualizer = None
        super(MultiBodyTreeEnv, self).__init__()

    def init_visualizer(self):
        if self._visualizer is None:
            self._visualizer = MeshcatVisualizerMBP(self.scene_graph)
            self._visualizer.load()

    @property
    def visualizer(self):
        return self._visualizer

    def plant_system(self):
        '''
        Implements the get_mdp_diagram method in DrakeEnv by constructing a RigidBodyPlant
        '''

        # Add Systems
        builder = DiagramBuilder()
        self.scene_graph = builder.AddSystem(SceneGraph())
        self.mbp = builder.AddSystem(MultibodyPlant())

        # Load the model from the file
        AddModelFromSdfFile(
            file_name=self.fname, plant=self.mbp, scene_graph=self.scene_graph)
        self.mbp.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
        self.mbp.Finalize(self.scene_graph)
        assert self.mbp.geometry_source_is_registered()

        # Visualizer must be initialized after Finalize() and before CreateDefaultContext()
        self.init_visualizer()

        builder.Connect(
            self.mbp.get_geometry_poses_output_port(),
            self.scene_graph.get_source_pose_port(self.mbp.get_source_id()))

        # Expose the inputs and outputs and build the diagram
        self._input_port_index_action = builder.ExportInput(
            self.mbp.get_actuation_input_port())
        self._output_port_index_state = builder.ExportOutput(
            self.mbp.get_continuous_state_output_port())
        self.diagram = builder.Build()

        return self.diagram

    def get_input_port_action(self):
        '''
        Returns the system input port that corresponds to the action
        '''
        return self.diagram.get_input_port(self._input_port_index_action)

    def get_output_port_observation(self):
        '''
        Returns the system output port that corresponds to the observation
        '''
        raise self.diagram.get_output_port(self._output_port_index_state)

    @property
    def action_space(self):
        return spaces.Box(*self.action_limits)

    @property
    def observation_space(self):
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

    def render(self, mode='human', close=False):
        '''
        Sends an LCM message to the visualizer
        '''
        sim_context = self.simulator.get_context()
        sg_context = context = self.diagram.GetSubsystemContext(
            self.scene_graph, sim_context)
        self.visualizer.draw(sg_context)
