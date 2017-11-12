from rl.learning import *

it = InputThread()
configurations = [
    LearningThread(dict(
        save_dir='firstConf'
    )),
]
it.start()
for conf in configurations:
    conf.start()

it.join()
for conf in configurations:
    conf.join()




# while True:
#     state = env.reset()
#     agent.reset()
#     timestep = 0
#     while True:
#         action = agent.act(states=state)
#         terminal = False
#         for _ in range(REPEAT_ACTIONS):
#             state, terminal, reward = env.execute(actions=action)
#             timestep += 1
#             env.gym.render()
#         if terminal is True or timestep == MAX_EPISODE_TIMESTEPS:
#             timestep = 0
#             break