import gym
import numpy as np
from pydrake.all import DiagramBuilder, SceneGraph, MultibodyPlant, AddModelFromSdfFile, UniformGravityFieldElement
from gym_drake.envs import drake_env
from meshcat_visualizer import MeshcatVisualizer

class MultiBodyTreeEnv(drake_env.DrakeEnv):
    '''
    Implements a DrakeEnv for models specified by a RigidBodyTree. Constructs
    the RigidBodyPlant for simulation and DrakeVisualizer for visualization.
    '''

    def __init__(self, fname):
        self.fname = fname
        self._visualizer = None
        super(MultiBodyTreeEnv, self).__init__()

    @property
    def visualizer(self):
        if self._visualizer is None:
            self._visualizer = MeshcatVisualizer(self.scene_graph)
        return self._visualizer

    def plant_system(self):
        '''
        Implements the get_mdp_diagram method in DrakeEnv by constructing a RigidBodyPlant
        '''
        builder = DiagramBuilder()
        self.scene_graph = builder.AddSystem(SceneGraph())
        self.mbp = builder.AddSystem(MultibodyPlant())
        AddModelFromSdfFile(
            file_name=self.fname, plant=self.mbp, scene_graph=self.scene_graph)
        # mbp.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
        self.mbp.Finalize(self.scene_graph)
        assert self.mbp.geometry_source_is_registered()
        # self.np = mbp.num_positions()
        # self.nv = mbp.num_velocities()

        builder.Connect(
            self.mbp.get_geometry_poses_output_port(),
            self.scene_graph.get_source_pose_port(self.mbp.get_source_id()))

        # visualize = disableViewer in kwargs and kwargs[disableViewer]
        # if visualize:
        visualizer = builder.AddSystem(MeshcatVisualizer(self.scene_graph))
        visualizer.load() # TODO: do we need this?
        
        builder.Connect(self.scene_graph.get_pose_bundle_output_port(),
                        visualizer.get_input_port(0))

        self._input_port_index_action = builder.ExportInput(self.mbp.get_actuation_input_port())
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
        raise NotImplementedError


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

    # def init_visualizer(self):
    #     return DrakeVisualizer(tree_)

