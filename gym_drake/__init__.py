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
    id='DrakeAcrobotRBTEnv-v0',
    entry_point='gym_drake.envs:AcrobotRBTEnv',
)

register(
    id='DrakeAcrobotMBT-v0',
    entry_point='gym_drake.envs:AcrobotMBTEnv',
)

register(
    id='DrakeCartpoleMBT-v0',
    entry_point='gym_drake.envs:CartpoleMBTEnv',
)
