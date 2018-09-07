import gym
import numpy as np
from pydrake.all import (RigidBodyTree, RigidBodyFrame,
                         AddModelInstanceFromUrdfFile, FloatingBaseType)
from rigid_body_tree_env import RigidBodyTreeEnv


class AcrobotRBTEnv(RigidBodyTreeEnv):
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
        RigidBodyTreeEnv.__init__(self, tree)

    @property
    def action_limits(self):
        return self._action_limits

    @property
    def observation_limits(self):
        return self._observation_limits

if __name__ == "__main__":
    acrobot = AcrobotEnv()
    acrobot.reset()
    acrobot.step()