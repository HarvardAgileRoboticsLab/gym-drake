from gym.envs.registration import register

register(
    id='drake-v0',
    entry_point='gym_drake.envs:DrakeEnv',
)

register(
    id='RigidBodyTree-v0',
    entry_point='gym_drake.envs:RigidBodyTree',
)

register(
    id='MultiBodyPlant-v0',
    entry_point='gym_drake.envs:MultiBodyPlant',
)

register(
    id='DrakeAcrobotRBT-v0',
    entry_point='gym_drake.envs:AcrobotRBTEnv',
)

register(
    id='DrakeAcrobotMBP-v0',
    entry_point='gym_drake.envs:AcrobotMBPEnv',
)

register(
    id='DrakeCartpoleMBP-v0',
    entry_point='gym_drake.envs:CartpoleMBPEnv',
)
