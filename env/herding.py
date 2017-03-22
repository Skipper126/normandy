import gym
import random
from gym.envs.classic_control import rendering
from gym import spaces
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
        #Tutaj Michał jedziesz
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
        self.sheepList = []
        self.wolfList = []
        for _ in range(self.sheepCount):
            self.sheepList.append(Sheep())
        for _ in range(self.wolfCount):
            self.wolftList.append(Wolf())

        self.action_space = spaces.MultiDiscrete([-5, 5], [-5, 5])

    def _step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        for sheep in self.sheepList:
            sheep.move()

    def _reset(self):
        pass

    def _render(self, mode='human', close=False):

        if self.viewer is None:
            screenWidth = 600
            screenHeight = 400
            self.viewer = rendering.Viewer(screenWidth, screenHeight)

            for agent in self.sheepList + self.wolfList:
                self.viewer.add_geom(agent.getBody())

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

