import gym
import numpy as np
# from pydrake.all import (MultiBodyTree, RigidBodyFrame,
#                          AddModelInstanceFromUrdfFile, FloatingBaseType)
# from pydrake.multibody.multibody_tree import UniformGravityFieldElement
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
# from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from multi_body_tree_env import MultiBodyTreeEnv
# from underactuated import MeshcatVisualizer  # TODO


class AcrobotMBTEnv(MultiBodyTreeEnv):
    def __init__(self, limits=None):
        fname = "models/Acrobot.urdf"

        # Set limits
        if limits is None:
            limits = {
                'action': (np.array([-2.0]), np.array([-2.0])),
                'observation': (np.array([-np.inf, -np.inf]),
                                np.array([np.inf, np.inf]))
            }

        # Call super-class constructor
        super(AcrobotMBTEnv, self).__init__(fname, limits)


if __name__ == "__main__":
    AcrobotMBTEnv()