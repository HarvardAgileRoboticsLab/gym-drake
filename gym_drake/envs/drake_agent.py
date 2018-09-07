import gym
import gym_drake
import numpy as np
import sys

if __name__ == '__main__':
    if len(sys.argv) > 1:
        env = gym.make(sys.argv[1])
    else:
        # env = gym.make('DrakeAcrobotMBT-v0')
        env = gym.make('DrakeCartpoleMBT-v0')

    # env.disableViewer = True

    env.reset()


    for i in range(20):
        env.step([0])
        env.render()
        print("step ",i)
        # env.step(env.action_space.sample())
