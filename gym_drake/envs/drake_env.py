import gym
from gym import error, spaces, utils

from pydrake.all import Simulator, DiagramBuilder, MdpDiagram


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

        assert len(input_limit_low) == len(input_limit_low)
        assert len(input_limit_low) == system.get_num_total_inputs()
        assert len(output_limit_low) == len(output_limit_low)
        assert len(output_limit_low) == system.get_num_total_outputs()

        # Define the action and observation space.
        self.action_space = spaces.Box(*self.action_limits)
        self.observation_space = spaces.Box(*self.observation_limits)

        # Create the diagram.
        self.builder = DiagramBuilder()
        initial_input = np.zeros_like(self.input_limit_low)
        self.input = builder.AddSystem(ConstantVectorSource(initial_input))
        self.system = builder.AddSystem(self.get_mdp_diagram())
        builder.Connect(self.input.get_output_port(),
                        self.system.get_input_port_action())
        self.diagram = builder.Build()

        # Create the simulator.
        self.simulator = Simulator(self.diagram)
        self.time = 0.

    def get_mdp_diagram(self):
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

    @property
    def action_limits(self):
        '''
        Specifies the limits of tha action space. This must be overridden by subclasses.
        '''
        raise NotImplementedError

    @property
    def observation_limits(self):
      '''
      Specifies the limits of tha action space. This must be overridden by subclasses.
      '''
      raise NotImplementedError

    def step(self, action):
        '''
        Simulates the system diagram for a short period of time
        '''
        temp = self.input_system.get_mutable_source_value().get_mutable_value()
        temp = action
        simulator.StepTo(self.time + self.dt)
        self.time += self.dt
        pass

    def reset(self):
        self.time = 0
        '''
        Resets the state in the system diagram
        '''
        pass

    def render(self, mode='human', close=False):
        '''
        Sends an LCM message to the visualizer
        '''
        pass
