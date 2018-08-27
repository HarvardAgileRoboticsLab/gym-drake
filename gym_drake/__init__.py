from gym.envs.registration import register

register(
    id='drake-v0',
    entry_point='gym_drake.envs:DrakeEnv',
)
