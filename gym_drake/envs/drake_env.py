import gym
from gym import error, spaces, utils
import numpy as np

from pydrake.all import Simulator, DiagramBuilder, ConstantVectorSource#, MdpDiagram
# from pydrake.systems.framework import DiagramBuilder

class DrakeEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        '''
        Sets up the System diagram and creates a drake visualizer object to
        send LCM messages to during the render method.

        Subclasses must implement the variables
        dt                : timestep between step() calls
        input_limit_low   : lower bound on inputs
        input_limit_high  : upper bound on inputs
        output_limit_low  : lower bound on outputs (may be -np.inf)
        output_limit_high : upper bound on outputs (may be np.inf)
        '''

        # Create the Diagram.
        self.system = self.plant_system()
        self.context = self.system.CreateDefaultContext()
        # zero_ctrl = np.zeros(self.system.get_input_port_action().size())
        # action_fixed_input_port_value = self.context.FixInputPort(self.get_input_port_action().get_index(), zero_ctrl)
        # self.mutable_action_vector = action_fixed_input_port_value.GetMutableVectorData()

        # assert len(input_limit_low) == len(input_limit_low)
        # assert len(input_limit_low) == self.system.input_port_action().size()
        # assert len(output_limit_low) == len(output_limit_low)
        # assert len(output_limit_low) == self.output_port_observation().size()

        # Create the simulator.
        self.simulator = Simulator(self.system, self.context)
        self.simulator.set_publish_every_time_step(False)
        print('drake env init')

    def plant_system(self):
        '''
        Returns the fully constructed MDP diagram which is connected to the
        vector source for simulation. Each subclass should implement this
        method.
        '''
        raise NotImplementedError

    def init_visualizer(self):
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
        # temp = self.input_system.get_mutable_source_value().get_mutable_value()
        # temp = action
        self.simulator.StepTo(self.context.get_time() + self.dt)

    def reset(self):
        '''
        Resets the state in the system diagram
        '''
        pass

    def render(self, mode='human', close=False):
        '''
        Sends an LCM message to the visualizer
        '''
        pass
