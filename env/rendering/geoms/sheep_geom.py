from .geom import Geom
from gym.envs.classic_control import rendering


class SheepGeom(Geom):
    BODY = 0

    def _createBody(self):
        body = rendering.make_circle(self.object.radius, res=50)
        body.set_color(181 / 255, 185 / 255, 215 / 255)
        self.geomPartList.append(body)

    def update(self):
        tr = self.geomPartTransformList
        tr[self.BODY].set_translation(self.object.x, self.object.y)