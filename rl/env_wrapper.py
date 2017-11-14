from env.herding import Herding, RotationMode
from tensorforce.contrib.openai_gym import OpenAIGym
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