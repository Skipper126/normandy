from gym.envs.classic_control import rendering
from .geoms.sheep_geom import SheepGeom
from .geoms.dog_geom import DogGeom
from ..constants import EnvParams


class Renderer:

    def __init__(self, sheepList, dogList, envParams: EnvParams):
        self.params = envParams
        self.mapWidth = self.params.MAP_WIDTH
        self.mapHeight = self.params.MAP_HEIGHT
        self.dogList = dogList
        self.sheepList = sheepList
        self.geomList = []
        self.viewer = rendering.Viewer(self.mapWidth, self.mapHeight)
        self._initRenderObjects()

    def _initRenderObjects(self):
        for sheep in self.sheepList:
            self.geomList.append(SheepGeom(sheep, self.params))

        for dog in self.dogList:
            self.geomList.append(DogGeom(dog, self.params))

        for geom in self.geomList:
            self.viewer.geoms.extend(geom.geomPartList)

    def render(self):
        for geom in self.geomList:
            geom.update()

        self.viewer.render()

    def close(self):
        self.viewer.close()
