from env.herding import Herding, RotationMode
from tensorforce.contrib.openai_gym import OpenAIGym
import numpy as np
from gym import spaces


class OpenAIWrapper(OpenAIGym):

    def __init__(self, env, gym_id):
        self.gym_id = gym_id
        self.gym = env


class EnvWrapper(Herding):

    def _step(self, action):
        state, reward, terminal, _ = super()._step(action)
        newState = []
        for i, _ in enumerate(self.dogList):
            s = state[i]
            s = s.flatten()
            newState.append(s)

        return newState, reward, terminal, _

    def _reset(self):
        state = super()._reset()
        newState = []
        for i, _ in enumerate(self.dogList):
            s = state[i]
            s = s.flatten()
            newState.append(s)

        return newState

    def _checkIfDone(self):
        if self.scatter < self.params.SCATTER_LEVEL:
            return True

        if self.dogList[0].y > self.mapHeight + 100 or self.dogList[0].y < -100 or \
                self.dogList[0].x > self.mapWidth + 100 or self.dogList[0].x < -100:
                return True
        if np.abs(self.dogList[0].y - self.herdCentrePoint[1]) < 5 and \
                np.abs(self.dogList[0].x - self.herdCentrePoint[0]) < 5:
                return True
        return False

    def _reward(self):
        self._scatter()
        returnValue = 0
        if self.scatter < self.previousScatter:
            returnValue = self.params.REWARD
        elif self.scatter <= self.params.SCATTER_LEVEL:
            returnValue = self.params.REWARD_FOR_HERDING
        else:
            returnValue = -self.params.REWARD

        return returnValue

    @property
    def action_space(self):
        dim = 3 if self.rotationMode is RotationMode.FREE else 2
        singleActionSpace = spaces.Box(-1, 1, (dim,))
        return singleActionSpace

    @property
    def observation_space(self):
        return spaces.Box(-1, 1, ((self.raysCount + 1) * 2,))

    def _close(self):
        pass