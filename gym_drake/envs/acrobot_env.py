import gym
import numpy as np
from pydrake.all import (RigidBodyTree, RigidBodyFrame,
                         AddModelInstanceFromUrdfFile, FloatingBaseType)
from rigid_body_tree_env import RigidBodyTreeEnv


class AcrobotEnv(RigidBodyTreeEnv):
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
            limits = {
                'action': (np.array([-2.0]), np.array([-2.0])),
                'observation': (np.array([-np.inf, -np.inf]),
                                np.array([np.inf, np.inf]))
            }

        # Call super-class constructor
        super(AcrobotEnv, self).__init__(tree, limits)


if __name__ == "__main__":
    AcrobotEnv()