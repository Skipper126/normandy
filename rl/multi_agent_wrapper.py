import numpy as np
from tensorforce import Configuration


class MultiAgentWrapper:

    def __init__(self, agentType, agentParams, agentsCount):
        self.agents = []
        firstAgent = agentType(**agentParams)
        self.agents.append(firstAgent)
        self.model = firstAgent.model
        for _ in range(agentsCount - 1):
            agent = agentType(**agentParams)
            agent.model.close()
            agent.model = self.model
            self.agents.append(agent)

    def reset(self):
        for agent in self.agents:
            agent.reset()

    def close(self):
        self.model.close()
    @property
    def timestep(self):
        return self.agents[0].timestep

    @property
    def episode(self):
        return self.agents[0].episode

    def act(self, states, deterministic=False):
        action = ()
        for i, agent in enumerate(self.agents):
            action += (agent.act(states=states[i], deterministic=deterministic),)
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
