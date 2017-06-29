from gym.envs.classic_control import rendering
from .geoms import *
from ..herding import Herding


class Renderer:

    def __init__(self, env: Herding):
        self.env = env
        self.params = env.params
        self.mapWidth = self.params.MAP_WIDTH
        self.mapHeight = self.params.MAP_HEIGHT
        self.geomList = []
        self.viewer = rendering.Viewer(self.mapWidth, self.mapHeight)
        self._initRenderObjects()
        for geom in self.geomList:
            self.viewer.geoms.extend(geom.geomPartList)

    def _initRenderObjects(self):
        for sheep in self.env.sheepList:
            self.geomList.append(sheep_geom.SheepGeom(self.env, sheep))

        for dog in self.env.dogList:
            self.geomList.append(dog_geom.DogGeom(self.env, dog))

        self.geomList.append(crosshair.Crosshair(self.env))

    def render(self):
        for geom in self.geomList:
            geom.update()

        self.viewer.render()

    def close(self):
        self.viewer.close()
