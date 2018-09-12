import gym
import gym_drake
import numpy as np
import sys, time

if __name__ == '__main__':
    if len(sys.argv) > 1:
        env = gym.make(sys.argv[1])
    else:
        # env = gym.make('DrakeAcrobotMBP-v0')
        env = gym.make('DrakeCartpoleMBP-v0')

    env.reset()

    for i in range(20):
        env.step(env.action_space.sample())
        env.render()
        time.sleep(.1)
        print("step ",i)
