import gym
from gym import error, spaces, utils
from gym.utils import seeding

from pydrake.all import Simulator, DiagramBuilder

class DrakeEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, system):
    '''
    Sets up the System diagram and creates a drake visualizer object to send LCM messages to during the render method.
    
    Subclasses must implement the variables
    dt                : timestep between step() calls
    input_limit_low   : lower bound on inputs
    input_limit_high  : upper bound on inputs
    output_limit_low  : lower bound on outputs (may be -np.inf)
    output_limit_high : upper bound on outputs (may be np.inf)
    '''

    assert len(input_limit_low) == len(input_limit_low)
    assert len(input_limit_low) == system.get_num_total_inputs()
    assert len(output_limit_low) == len(output_limit_low)
    assert len(output_limit_low) == system.get_num_total_outputs()

    self.action_space = spaces.Box(self.input_limit_low, self.input_limit_high)
    self.observation_space = spaces.Box(self.output_limit_low, self.output_limit_high)

    self.builder = DiagramBuilder()

    # Connect a source
    initial_input = np.zeros_like(self.input_limit_low)
    self.input_system = builder.AddSystem(ConstantVectorSource(initial_input))
    self.input = 

    self.controlled_system = system
    self.system = builder.Build()
    self.simulator = Simulator(self.system)
    self.time = 0.

  def step(self, action):
    '''
    Simulates the system diagram for a short period of time
    '''
    self.input_system.get_mutable_source_value().get_mutable_value() = action
    simulator.StepTo()
    pass

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
