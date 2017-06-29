from .geom import Geom
from gym.envs.classic_control import rendering
import math


class DogGeom(Geom):
    BODY = 0
    RAY = 1
    COLOR = {
        -1: (1, 0, 0),
        0: (0, 0, 0),
        1: (0, 1, 0)
    }

    def __init__(self, env, dogObject):
        self.object = dogObject
        self.params = env.params
        super().__init__(env)

    def _createBody(self):

        body = rendering.make_circle(self.object.radius, res=50)
        body.set_color(185 / 255, 14 / 255, 37 / 255)
        self.geomPartList.append(body)

        for _ in range(self.params.RAYS_COUNT):
            line = rendering.Line((0, 0), (self.params.RAY_LENGTH, 0))
            self.geomPartList.append(line)

    def update(self):
        tr = self.geomPartTransformList
        tr[self.BODY].set_translation(self.object.x, self.object.y)
        for i in range(self.params.RAYS_COUNT):
            tr[self.RAY + i].set_scale(self.object.observation[0][i], 0)
            color = tuple(min(x*(1.5 - self.object.observation[0][i]), 1) for x in self.COLOR[self.object.observation[1][i]])
            self.geomPartList[self.RAY + i].set_color(*color)
            rot = self.object.rotation - self.object.rayRadian[i]
            tr[self.RAY + i].set_rotation(rot)
            x = math.cos(rot) * self.object.radius
            y = math.sin(rot) * self.object.radius
            tr[self.RAY + i].set_translation(self.object.x + x, self.object.y + y)
