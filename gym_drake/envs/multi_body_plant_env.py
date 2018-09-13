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
from utils import get_full_model_path

class MultiBodyPlantEnv(drake_env.DrakeEnv):
    '''
    Implements a DrakeEnv for models specified by a MultiBodyPlant. Constructs
    the MultiBodyPlant for simulation and MeshcatVisualizer for visualization.
    '''

    def __init__(self, model_path):
        self.model_path = get_full_model_path(model_path)
        self._visualizer = None
        super(MultiBodyPlantEnv, self).__init__()

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
            file_name=self.model_path, plant=self.mbp, scene_graph=self.scene_graph)
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

        self._output = self.mbp.AllocateOutput()
        return self.diagram

    def get_input_port_action(self):
        '''
        Returns the system input port that corresponds to the action
        '''
        return self.diagram.get_input_port(self._input_port_index_action)

    def get_observation(self):
        return self.get_state()

    def get_state(self):
        '''
        Returns the system output port that corresponds to the observation
        '''
        sim_context = self.simulator.get_context()
        mbp_context = self.diagram.GetSubsystemContext(
            self.mbp, sim_context)
        return mbp_context.get_continuous_state().get_vector().get_value()

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
        Sends an LCM message to the visualizer
        '''
        sim_context = self.simulator.get_context()
        sg_context = self.diagram.GetSubsystemContext(
            self.scene_graph, sim_context)
        self.visualizer.draw(sg_context)
