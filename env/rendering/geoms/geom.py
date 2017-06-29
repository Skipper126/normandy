from gym.envs.classic_control import rendering
from ...herding import Herding

class Geom:
    def __init__(self, env :Herding):
        self.env = env
        self.geomPartList = []
        self.geomPartTransformList = []
        self._createBody()
        self._addTransformToAllParts()

    def _createBody(self):
        raise NotImplementedError

    def _addTransformToAllParts(self):
        for part in self.geomPartList:
            transform = rendering.Transform()
            self.geomPartTransformList.append(transform)
            part.add_attr(transform)

    def update(self):
        raise NotImplementedError
