import gym
from gym import (error, spaces, utils)
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    Simulator,
)


class DrakeEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        '''
        Sets up the System diagram and creates a drake visualizer object to
        send LCM messages to during the render method.

        Subclasses must implement the methods:
            - plant_system()
            - visualizer()
            - get_input_port_action()
            - get_output_port_observation()
            - action_space()
            - observation_space()
            - render()
        '''

        # Create the Diagram.
        self.system = self.plant_system()
        self.context = self.system.CreateDefaultContext()

        # Create the simulator.
        self.simulator = Simulator(self.system, self.context)
        self.simulator.set_publish_every_time_step(False)

    def plant_system(self):
        '''
        Returns the fully constructed MDP diagram which is connected to the
        vector source for simulation. Each subclass should implement this
        method.
        '''
        raise NotImplementedError

    @property
    def visualizer(self):
        '''
        Initializes and returns a DrakeVisualizer system. This must be overridden by subclasses.
        '''
        raise NotImplementedError

    def get_input_port_action(self):
        '''
        Returns the system input port that corresponds to the action
        '''
        raise NotImplementedError

    def get_output_port_observation(self):
        '''
        Returns the system output port that corresponds to the observation
        '''
        raise NotImplementedError

    @property
    def action_space(self):
        '''
        Specifies the limits of tha action space. This must be overridden by subclasses.
        '''
        raise NotImplementedError

    @property
    def observation_space(self):
        '''
        Specifies the limits of tha action space. This must be overridden by subclasses.
        '''
        raise NotImplementedError

    def step(self, action):
        '''
        Simulates the system diagram for a short period of time
        '''
        action_fixed_input_port_value = self.context.FixInputPort(
            self.get_input_port_action().get_index(), action)
        self.simulator.StepTo(self.context.get_time() + self.dt)
        # return observation, reward, done, info

    def reset(self):
        '''
        Resets the state in the system diagram
        '''
        self.context.set_time(0)
        pass

    def render(self, mode='human', close=False):
        '''
        Notifies the visualizer to draw the current state. Different implementations based on MultiBodyPlant and RigidBodyTree
        '''
        raise NotImplementedError