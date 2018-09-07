import gym
import gym_drake
import numpy as np
import sys

if __name__ == '__main__':
    if len(sys.argv) > 1:
        env = gym.make(sys.argv[1])
    else:
        env = gym.make('DrakeAcrobotMBT-v0')
        # env = gym.make('DrakeCartpole-v0')

    env.env.disableViewer = True

    env.reset()


    for i in range(20):
        env.step([0])
        print("step ",i)
        # env.step(env.action_space.sample())

    # env.render(mode='graph')

    # env.render(close=True)