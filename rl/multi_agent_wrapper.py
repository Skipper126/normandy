import numpy as np
from tensorforce import Configuration


class MultiAgentWrapper:

    def __init__(self, config, Agent, quantity):
        self.agents = []
        firstAgent = Agent(Configuration(**config))
        self.agents.append(firstAgent)
        self.model = firstAgent.model
        for _ in range(quantity - 1):
            agent = Agent(Configuration(**config), self.model)
            self.agents.append(agent)

    def reset(self):
        for agent in self.agents:
            agent.reset()

    def act(self, state, deterministic=False):
        action = ()
        for i, agent in enumerate(self.agents):
            s = np.append(state[i][0], state[i][1])
            action += (agent.act(state=s, deterministic=deterministic),)
        return action

    def observe(self, reward, terminal):
        for agent in self.agents:
            agent.observe(reward=reward, terminal=terminal)

    def observe_episode_reward(self, episode_reward):
        self.model.write_episode_reward_summary(episode_reward)

    def load_model(self, path):
        self.model.load_model(path)

    def save_model(self, path):
        self.model.save_model(path)
