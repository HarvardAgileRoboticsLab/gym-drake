import gym
import numpy as np
from pydrake.all import (RigidBodyTree, RigidBodyFrame,
                         AddModelInstanceFromUrdfFile, FloatingBaseType)
from gym_drake.envs import rigid_body_tree_env


class AcrobotRBTEnv(rigid_body_tree_env.RigidBodyTreeEnv):
    def __init__(self, limits=None):
        # create RigidBodyTree
        tree = RigidBodyTree()
        world_frame = RigidBodyFrame("world_frame", tree.world(), [0, 0, 0],
                                     [0, 0, 0])
        model_path = "models/Acrobot.urdf"
        AddModelInstanceFromUrdfFile(model_path, FloatingBaseType.kFixed,
                                     world_frame, tree)
        # Set limits
        if limits is None:
            self._action_limits = (np.array([-2.0]), np.array([-2.0]))
            self._observation_limits = (np.array([-np.inf, -np.inf]), np.array([np.inf, np.inf]))
        else:
            self._action_limits = limits['action']
            self._observation_limits = limits['observation']

        # Call super-class constructor
        rigid_body_tree_env.RigidBodyTreeEnv.__init__(self, tree)

    @property
    def action_limits(self):
        return self._action_limits

    @property
    def observation_limits(self):
        return self._observation_limits
    

if __name__ == "__main__":
    model_path = "models/Acrobot.urdf"
    tree = RigidBodyTree()
    world_frame = RigidBodyFrame("world_frame", tree.world(), [0, 0, 0],
                                 [0, 0, 0])
    model_path = "models/Acrobot.urdf"
    AddModelInstanceFromUrdfFile(model_path, FloatingBaseType.kFixed,
                                 world_frame, tree)
    builder = DiagramBuilder()
    plant = builder.AddSystem(RigidBodyPlant(tree))
    visualizer = MeshcatRigidBodyVisualizer(tree, draw_collision=True)

    nx = tree.get_num_positions() + tree.get_num_velocities()

    torque_system = builder.AddSystem(ConstantVectorSource(
                            np.ones(tree.get_num_actuators(), 1)))
    builder.Connect(torque_system.get_output_port(0),
                    plant_sys.get_input_port(0))

    # Visualize
    visualizer = builder.AddSystem(visualizer)
    builder.Connect(plant_sys.get_output_port(0),
                    visualizer.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # TODO(russt): Clean up state vector access below.
    state = simulator.get_mutable_context().get_mutable_state()\
                     .get_mutable_continuous_state().get_mutable_vector()

    if set_initial_state:
        initial_state = np.zeros((nx, 1))
        initial_state[0] = 1.0
        state.SetFromVector(initial_state)

    simulator.StepTo(3.0)

    # if (args.animate):
    #     # Generate an animation of whatever happened
    #     ani = visualizer.animate(signalLogger)