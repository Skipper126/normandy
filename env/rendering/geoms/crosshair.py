from .geom import Geom
from gym.envs.classic_control import rendering


class Crosshair(Geom):

    def _createBody(self):
        crosshairSize = 10
        verticalBar = rendering.Line((-crosshairSize - 1, 0), (crosshairSize, 0))
        horizontalBar = rendering.Line((0, -crosshairSize - 1), (0, crosshairSize))

        verticalBar.set_color(0, 0, 0)
        horizontalBar.set_color(0, 0, 0)

        self.geomPartList.append(verticalBar)
        self.geomPartList.append(horizontalBar)

    def update(self):
        tr = self.geomPartTransformList
        tr[0].set_translation(self.env.herdCentreX, self.env.herdCentreY)
        tr[1].set_translation(self.env.herdCentreX, self.env.herdCentreY)
