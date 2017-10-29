from env.herding import Herding, EnvParams
import numpy as np
from tensorforce.agents import TRPOAgent
from tensorforce.core.networks import layered_network_builder
from tensorforce.execution import Runner
from tensorforce.contrib.openai_gym import OpenAIGym
from rl.multi_agent_wrapper import MultiAgentWrapper


class OpenAIWrapper(OpenAIGym):

    def __init__(self, env, gym_id):
        self.gym_id = gym_id
        self.gym = env


params = EnvParams()
params.DOG_COUNT = 3
params.SHEEP_COUNT = 10
params.RAYS_COUNT = 128
params.FIELD_OF_VIEW = 180
env = OpenAIWrapper(Herding(params), 'herding')

agent = MultiAgentWrapper(config=dict(
    log_level='info',
    batch_size=4096,

    gae_lambda=0.97,
    learning_rate=0.001,
    entropy_penalty=0.01,
    epochs=5,
    optimizer_batch_size=512,
    loss_clipping=0.2,

    states=env.states,
    actions=env.actions,
    network=layered_network_builder([
        dict(type='dense', size=32),
        dict(type='dense', size=32)
    ])
), Agent=TRPOAgent, quantity=params.DOG_COUNT)

# Create the runner
runner = Runner(agent=agent, environment=env)


# Callback function printing episode statistics
def episode_finished(r):
    print("Finished episode {ep} after {ts} timesteps (reward: {reward})".format(ep=r.episode, ts=r.timestep,
                                                                                 reward=r.episode_rewards[-1]))
    return True


# Start learning
runner.run(episodes=10, episode_finished=episode_finished)

while True:
    state = env.reset()
    terminal = False
    while terminal is False:
        action = agent.act(state=state, deterministic=True)
        state, reward, terminal = env.execute(action)
        env.gym.render()