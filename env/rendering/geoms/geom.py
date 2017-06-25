from gym.envs.classic_control import rendering
from ...constants import EnvParams

class Geom:
    def __init__(self, envObject, envParams: EnvParams):
        self.params = envParams
        self.object = envObject
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
