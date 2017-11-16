import sys
import os
from env.herding import EnvParams, AgentsLayout
from tensorforce.agents import TRPOAgent
from tensorforce.execution import Runner
from rl.multi_agent_wrapper import MultiAgentWrapper
from tensorforce import Configuration
from rl.env_wrapper import EnvWrapper, OpenAIWrapper
import threading
from statistics import mean

dog_count = 3
sheep_count = 30
batch_size = 2048


network_spec = [
    dict(type='dense', size=128),
    dict(type='dense', size=64)
]
params = EnvParams()
params.DOG_COUNT = dog_count
params.SHEEP_COUNT = sheep_count
env = OpenAIWrapper(EnvWrapper(params), 'herding')
configuration = Configuration(
    batch_size=batch_size,
    normalize_rewards=True
)
agent_type = TRPOAgent
agent = MultiAgentWrapper(
    agent_type,
    dict(
        states_spec=env.states,
        actions_spec=env.actions,
        network_spec=network_spec,
        config=configuration
    ),
    dog_count)
agent.load_model('./rl/model')

while True:
    state = env.reset()
    agent.reset()
    timestep = 0
    while True:
        action = agent.act(states=state)
        terminal = False
        for _ in range(5):
            state, terminal, reward = env.execute(actions=action)
            timestep += 1
            env.gym.render()
        if terminal is True or timestep == 50000:
            timestep = 0
            break