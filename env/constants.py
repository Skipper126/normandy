import math
import random

PI = math.pi
TWOPI = 2 * PI
DEG2RAD = 0.01745329252
EULER = math.e


class SheepBehaviour:
    SIMPLE, COMPLEX = range(2)


class RotationMode:
    FREE, LOCKED_ON_HERD_CENTRE = range(2)


class AgentsLayout:
    """
    AgentsLayout zawiera statyczne metody do rozstawienia agentów na przy starcie rundy
    """

    @staticmethod
    def RANDOM(env):
        padding = 5
        for agent in env.dogList + env.sheepList:
            x = random.randint(agent.radius + padding, env.mapWidth - agent.radius - padding)
            y = random.randint(agent.radius + padding, env.mapHeight - agent.radius - padding)
            agent.setPos(x, y)

    @staticmethod
    def DOGS_OUTSIDE_CIRCLE(env):
        padding = 5
        for agent in env.dogList:
            x = random.randint(agent.radius + padding, env.mapWidth - agent.radius - padding)
            y = random.randint(agent.radius + padding, env.mapHeight - agent.radius - padding)
            agent.setPos(x, y)

        wpadding = int(env.mapWidth / 6)
        hpadding = int(env.mapHeight / 6)
        for agent in env.sheepList:
            x = random.randint(agent.radius + wpadding, env.mapWidth - agent.radius - wpadding)
            y = random.randint(agent.radius + hpadding, env.mapHeight - agent.radius - hpadding)
            agent.setPos(x, y)


    @staticmethod
    def DOGS_INSIDE_CIRCLE(env):
        # TODO
        pass


class EnvParams:

    def __init__(self):
        self.SHEEP_COUNT = 7
        self.DOG_COUNT = 2
        self.AGENT_RADIUS = 15
        self.SHEEP_BEHAVIOUR = SheepBehaviour.SIMPLE
        self.FIELD_OF_VIEW = 180
        self.RAYS_COUNT = 128
        self.RAY_LENGTH = 300
        self.MAX_MOVEMENT_DELTA = 5
        self.MAX_ROTATION_DELTA = 90
        self.MAP_HEIGHT = 800
        self.MAP_WIDTH = 1200
        self.LAYOUT_FUNCTION = AgentsLayout.RANDOM
        self.ROTATION_MODE = RotationMode.FREE
        self.REWARD_FOR_HERDING = 20
        self.EPOCH = 1000
        self.SCATTER_LEVEL = 200
