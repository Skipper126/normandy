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

    def __init__(self, x=None, y=None):
        Agent.__init__(x, y)
        self.rotation = 0

    def move(self, deltaX, deltaY, deltaRotation):
        self.x += deltaX
        self.y += deltaY
        self.rotation += deltaRotation
        self.transform.set_translation(self.x, self.y)
        self.transform.set_rotation(self.rotation)


class Herding(gym.Env):

    metadata = {
        'render.modes': ['human']
    }

    def __init__(self, params=None):
        self.viewer = None
        self.sheepCount = 4
        self.wolfCount = 1
        self.sheepList = []
        self.wolfList = []
        self.setParams(params)
        for _ in range(self.sheepCount):
            self.sheepList.append(Sheep())
        for _ in range(self.wolfCount):
            self.wolftList.append(Wolf())

        self.action_space = spaces.MultiDiscrete([-5, 5], [-5, 5], [0, 360])

    def _step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        for sheep in self.sheepList:
            sheep.move()
        #zakładamy że jest tylko jeden wilk
        deltaX = action[0]
        deltaY = action[1]
        deltaAngle = action[2]
        self.wolfList[0].move(deltaX, deltaY, deltaAngle)

        return [], 0, False, {}

    def _reset(self):
        return [], 0, False, {}

    def _render(self, mode='human', close=False):

        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        if self.viewer is None:
            screenWidth = 600
            screenHeight = 400
            self.viewer = rendering.Viewer(screenWidth, screenHeight)

            for agent in self.sheepList + self.wolfList:
                self.viewer.add_geom(agent.getBody())

        return self.viewer.render()

    def setParams(self,params):
        pass





def main():
    from pyglet.window import key
    x = y = 100

    #zbiór parametrów do środowiska. np. env = Herding(params)
    params = {
        'sheep count': 4,
        'wolf count': 1,
        'sheep behaviour': 'simple',
    }

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




if __name__ == "__main__":
    main()
