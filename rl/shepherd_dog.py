from env.herding import Herding, EnvParams
import numpy as np
from tensorforce.agents import TRPOAgent
from tensorforce.execution import Runner
from tensorforce.contrib.openai_gym import OpenAIGym
from rl.multi_agent_wrapper import MultiAgentWrapper
from tensorforce import Configuration


class OpenAIWrapper(OpenAIGym):

    def __init__(self, env, gym_id):
        self.gym_id = gym_id
        self.gym = env


params = EnvParams()
params.DOG_COUNT = 1
params.SHEEP_COUNT = 10
params.RAYS_COUNT = 128
params.FIELD_OF_VIEW = 180
params.MAX_MOVEMENT_DELTA = 2
params.EPOCH = 50000
params.MAX_ROTATION_DELTA = 20
env = OpenAIWrapper(Herding(params), 'herding')

agent = TRPOAgent(
    states_spec=env.states,
    actions_spec=env.actions,
    network_spec=[
        dict(type='dense', size=128),
        dict(type='dense', size=64)
    ],
    config=Configuration(
        batch_size=4096,
        normalize_rewards=True
    ))

# Create the runner
runner = Runner(agent=agent, environment=env)


# Callback function printing episode statistics
def episode_finished(r):
    print("Finished episode {ep} after {ts} timesteps (reward: {reward})".format(ep=r.episode, ts=r.timestep,
                                                                                 reward=r.episode_rewards[-1]))
    return True


# Start learning
runner.run(episode_finished=episode_finished, episodes=100, timesteps=2000)
input("continue")
env = OpenAIWrapper(Herding(params), 'herding')
while True:
    state = env.reset()
    terminal = False
    while terminal is False:
        action = agent.act(states=state, deterministic=True)
        print(action)
        state, terminal, reward = env.execute(action)
        env.gym.render()
