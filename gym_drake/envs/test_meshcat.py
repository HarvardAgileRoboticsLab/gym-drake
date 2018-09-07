from pydrake.all import (RigidBodyTree, RigidBodyPlant, RigidBodyFrame,
                         AddModelInstanceFromUrdfFile, FloatingBaseType, DiagramBuilder, ConstantVectorSource, Simulator)
from meshcat_rigid_body_visualizer import MeshcatRigidBodyVisualizer
import numpy as np

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
                        np.ones((tree.get_num_actuators(), 1))))
builder.Connect(torque_system.get_output_port(0),
                plant.get_input_port(0))

# Visualize
visualizer = builder.AddSystem(visualizer)
builder.Connect(plant.get_output_port(0),
                visualizer.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.Initialize()
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# TODO(russt): Clean up state vector access below.
state = simulator.get_mutable_context().get_mutable_state()\
                 .get_mutable_continuous_state().get_mutable_vector()

initial_state = np.zeros((nx, 1))
initial_state[0] = 1.0
state.SetFromVector(initial_state)

simulator.StepTo(3.0)