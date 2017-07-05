from .geom import *
from gym.envs.classic_control import rendering


class Crosshair(Geom):

    def __init__(self, herdCentrePoint):
        self.herdCentrePoint = herdCentrePoint
        crosshairSize = 10
        color = (0, 0, 0)
        self.verticalBar = Part(rendering.Line((-crosshairSize - 1, 0), (crosshairSize, 0)))
        self.horizontalBar = Part(rendering.Line((0, -crosshairSize - 1), (0, crosshairSize)))

        self.verticalBar.setColor(*color)
        self.horizontalBar.setColor(*color)

    def getParts(self):
        return [self.verticalBar.body, self.horizontalBar.body]

    def update(self):
        self.horizontalBar.setPos(self.herdCentrePoint[0], self.herdCentrePoint[1])
        self.verticalBar.setPos(self.herdCentrePoint[0], self.herdCentrePoint[1])
