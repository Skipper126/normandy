import gym
import random
from gym.envs.classic_control import rendering
import numpy as np


class Agent:

    def __init__(self, x=None, y=None):
        if x or y is None:
            self.x = random.randrange(0, 500)
            self.y = random.randrange(0, 500)
        else:
            self.x = x
            self.y = y

        self.transform.set_translation(self.x, self.y)
        self.transform = rendering.Transform()
        self.radius = 2
        self.body = rendering.make_circle(self.radius)
        self.body.add_attr(self.transform)


    def getBody(self):
        return self.body


class Sheep(Agent):

    def move(self):
        self.x += random.randrange(-5, 5)
        self.y += random.randrange(-5, 5)
        self.transform.set_translation(self.x, self.y)


class Wolf(Agent):

    def move(self, x, y):
        self.x = x
        self.y = y
        self.transform.set_translation(self.x, self.y)


class Herding(gym.Env):

    metadata = {
        'render.modes': ['human']
    }

    def __init__(self):
        self.viewer = None
        self.sheepCount = 4
        self.wolfCount = 1
        self.objectList = []

    def _step(self, action):
        pass

    def _reset(self):
        pass

    def _render(self, mode='human', close=False):

        if self.viewer is None:
            screenWidth = 600
            screenHeight = 400
            self.viewer = rendering.Viewer(screenWidth, screenHeight)

            for _ in range(self.sheepCount):
                sheep = Sheep()
                self.objectList.append(sheep)
                self.viewer.add_geom(sheep.getBody())

            wolf = Wolf()
            self.objectList.append(wolf)
            self.viewer.add_geom(wolf.getBody())

        return self.viewer.render()

    def _close(self):
        pass


if __name__ == "__main__":
    from pyglet.window import key
    x = y = 100

    def key_press(k, mod):
        global x, y
        if k == key.LEFT:
            x -= 5
        if k == key.RIGHT:
            x += 5
        if k == key.UP:
            y -= 5
        if k == key.DOWN:
            y += 5

