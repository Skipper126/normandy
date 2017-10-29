from .geom import *
from gym.envs.classic_control import rendering
import math


class DogGeom(Geom):

    COLOR = {
        -1: (1, 0, 0),
        0: (0, 0, 0),
        1: (0, 1, 0)
    }

    def __init__(self, dogObject, envParams):
        self.object = dogObject
        self.params = envParams

        self.body = Part(rendering.make_circle(self.object.radius, res=50))
        self.body.setColor(185 / 255, 14 / 255, 37 / 255)
        self.rays = []
        for _ in range(self.params.RAYS_COUNT):
            self.rays.append(Part(rendering.Line((0, 0), (self.params.RAY_LENGTH, 0))))

    def getParts(self):
        parts = [self.body.body]
        for ray in self.rays:
            parts.append(ray.body)
        return parts

    def update(self):
        self.body.setPos(self.object.x, self.object.y)
        for i, ray in enumerate(self.rays):
            ray.setScale(1 - self.object.observation[0][i], 0)
            color = tuple(min(x * (1.5 - self.object.observation[0][i]), 1) for x in self.COLOR[self.object.observation[1][i]])
            ray.setColor(*color)
            rot = self.object.rotation - self.object.rayRadian[i]
            ray.setRotation(rot)
            x = math.cos(rot) * self.object.radius
            y = math.sin(rot) * self.object.radius
            ray.setPos(self.object.x + x, self.object.y + y)
