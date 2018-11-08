import gym
from gym import (error, spaces)
from gym.utils import seeding
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

        Subclasses must implement the methods below that throw a
        NotImplementedError
        '''

        # Create the Diagram.
        self.system = self.plant_system()
        self.context = self.system.CreateDefaultContext()

        # Create the simulator.
        self.simulator = Simulator(self.system, self.context)
        self.simulator.set_publish_every_time_step(False)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def plant_system(self):
        '''
        Returns the fully constructed MDP diagram which is connected to the
        vector source for simulation. Each subclass should implement this
        method.
        '''
        raise NotImplementedError

    def get_input_port_action(self):
        '''
        Returns the system input port that corresponds to the action
        '''
        raise NotImplementedError

    def get_observation(self):
        '''
        Returns the system output port that corresponds to the observation
        '''
        raise NotImplementedError

    def get_state(self):
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

    def get_reward(self, state, action):
        '''
        Computes the reward from the state and an action
        '''
        raise NotImplementedError

    def reset_state(self):
        '''
        Computes the reward from the state and an action
        '''
        raise NotImplementedError

    def step(self, action):
        '''
        Simulates the system diagram for a short period of time
        '''
        # Only allow valid actions from the action space
        action = validate_action(action, self.action_space)
        
        # Fix input port with action and simulate
        action_fixed_input_port_value = self.context.FixInputPort(
            self.get_input_port_action().get_index(), action)
        self.simulator.StepTo(self.context.get_time() + self.dt)

        # Get the observation, reward, and terminal conditions
        state = self.get_state()
        observation = self.get_observation()
        reward = self.get_reward(state, action)
        done = self.is_done()
        info = {}   

        return observation, reward, done, info

    def reset(self):
        '''
        Resets the state in the system diagram
        '''
        self.context.set_time(0)
        self.reset_state()
        return self.get_observation()

    def render(self, mode='human', close=False):
        '''
        Notifies the visualizer to draw the current state. Different implementations based on MultiBodyPlant and RigidBodyTree
        '''
        raise NotImplementedError

def validate_action(action, action_space):
    '''
    Ensure the action is within the action space
    '''
    if type(action_space) is spaces.Box:
        action = np.minimum(action, action_space.high)
        action = np.maximum(action, action_space.low)

    return action
