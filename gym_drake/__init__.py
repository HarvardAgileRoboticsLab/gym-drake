from gym.envs.registration import register

register(
    id='drake-v0',
    entry_point='gym_drake.envs:DrakeEnv',
)

register(
    id='RigidBodyTree-v0',
    entry_point='gym_drake.envs:RigidBodyTree',
)

# Rob
# ----------------------------------------

register(
        id='DrakePendulum-v0',
    entry_point='gym_drake.envs:InvertedPendulumEnv',
    max_episode_steps=1000,
    reward_threshold=9100.0,
)

register(
        id='DrakeCartpole-v0',
    entry_point='gym_drake.envs:CartpoleEnv',
    max_episode_steps=1000,
    reward_threshold=9100.0,
)

register(
        id='DrakeAcrobotRBTEnv-v0',
    entry_point='gym_drake.envs:AcrobotRBTEnv',
    max_episode_steps=1000,
    reward_threshold=9100.0,
)
