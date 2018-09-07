import gym
import numpy as np
from pydrake.all import DiagramBuilder, MultibodyPlant, SceneGraph, UniformGravityFieldElement, AddModelFromSdfFile
from drake_env import DrakeEnv
from meshcat_visualizer import MeshcatVisualizer

class MultiBodyTreeEnv(DrakeEnv):
    '''
    Implements a DrakeEnv for models specified by a MultiBodyTree. Constructs
    the MultiBodyTree for simulation and MeshcatVisualizer for visualization.
    '''

    def __init__(self, fname, limits, **kwargs):
        self.dt = 0.01
        self.fname = fname
        self._visualizer = None

        self._action_limits = limits['action']
        self._observation_limits = limits['observation']
        print("mbt init")
        super(MultiBodyTreeEnv, self).__init__()

    @property
    def visualizer(self):
        if self._visualizer is None:
            self._visualizer = MeshcatVisualizer(self.scene_graph)
        return self._visualizer

    @property
    def action_limits(self):
        return self._action_limits

    @property
    def observation_limits(self):
        return self._observation_limits

    def get_mdp_diagram(self):
        '''
        Constructs the Environment-specific MDP
        '''
        builder = DiagramBuilder()
        self.scene_graph = builder.AddSystem(SceneGraph())
        mbp = builder.AddSystem(MultibodyPlant())
        AddModelFromSdfFile(
            file_name=self.fname, plant=mbp, scene_graph=self.scene_graph)
        mbp.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
        mbp.Finalize(self.scene_graph)
        assert mbp.geometry_source_is_registered()
        # self.np = mbp.num_positions()
        # self.nv = mbp.num_velocities()

        builder.Connect(
            mbp.get_geometry_poses_output_port(),
            self.scene_graph.get_source_pose_port(mbp.get_source_id()))

        # visualize = disableViewer in kwargs and kwargs[disableViewer]
        # if visualize:
        # visualizer = builder.AddSystem(MeshcatVisualizer(scene_graph))
        # builder.Connect(scene_graph.get_pose_bundle_output_port(),
        #                 visualizer.get_input_port(0))

        diagram = builder.Build()
        # if visualize:
        # visualizer.load() # TODO: is this re-orderable?
        return diagram


    def init_visualizer(self):
        return DrakeVisualizer(tree_)
