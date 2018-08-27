import gym
from gym import error, spaces, utils
from gym.utils import seeding

class DrakeEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self):
    '''
    Sets up the System diagram and creates a drake visualizer object to send LCM messages to during the render method.
    '''
    pass

  def step(self, action):
    '''
    Simulates the system diagram for a short period of time
    '''
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
