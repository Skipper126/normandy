import gym
from gym import spaces
import numpy as np
from .constants import *
from .agents.dog import Dog
from .agents.sheep import Sheep
from threading import Thread


class Herding(gym.Env):

    metadata = {
        'render.modes': ['human']
    }

    def __init__(self, params=None):
        self.params = EnvParams() if params is None else params
        self.mapHeight = self.params.MAP_HEIGHT
        self.mapWidth = self.params.MAP_WIDTH
        self.sheepCount = self.params.SHEEP_COUNT
        self.dogCount = self.params.DOG_COUNT
        self.maxMovementDelta = self.params.MAX_MOVEMENT_DELTA
        self.maxRotationDelta = self.params.MAX_ROTATION_DELTA
        self.raysCount = self.params.RAYS_COUNT
        self.setUpAgents = self.params.LAYOUT_FUNCTION
        self.sheepBehaviour = self.params.SHEEP_BEHAVIOUR
        self.rotationMode = self.params.ROTATION_MODE
        self.sheepList = []
        self.dogList = []
        self.viewer = None
        self.herdCentrePoint = [0, 0]
        self.previousScatter = 0
        self.scatter = 0
        self.rewardValue = 0
        self.constansScatterCounter = 0
        self.state = self._createState()
        self._createAgents()
        self.epoch = 0

    def _step(self, action):
        # assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        for i, dog in enumerate(self.dogList):
            # action[i] to deltaX, deltaY oraz opcjonalnie deltaRotation
            dog.move(action[i])

        for sheep in self.sheepList:
            sheep.move()

        for dog in self.dogList:
            dog.updateObservation()

        return self.state, self._reward(), self._checkIfDone(), {}

    def _reset(self):
        # Metoda statyczna klasy AgentsLayout. Wyjątkowo przyjmuje parametr self.
        self.setUpAgents(self)
        self.epoch = 0
        for dog in self.dogList:
            dog.updateObservation()

        return self.state

    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        """
        Wszystko, co dotyczy wyświetlania środowiska, jest tworzone przy pierwszym
        wywołaniu metody render().
        """
        if self.viewer is None:
            from .rendering.renderer import Renderer
            self.viewer = Renderer(self)

        self.viewer.render()

    def setAgentsLayoutFunction(self, layoutFunction):
        """
        Metoda ustawia funkcję to rozstawiania agentów przy wywołaniu reset()
        """
        self.setUpAgents = layoutFunction

    def setSheepBehaviourMode(self, behaviour):
        for sheep in self.sheepList:
            sheep.setBehaviourMode(behaviour)

    def _createAgents(self):
        for i in range(self.sheepCount):
            self.sheepList.append(Sheep(self.params))
        for i in range(self.dogCount):
            """
            Każdy z psów otrzymuje wskaźnik na fragment observation_space dotyczący jego
            obserwacji. Przy każdym kroku będzie aktualizował swój fragment tablicy.
            """
            self.dogList.append(Dog(self.state[i], self.params, self))

        # Każdy agent dostaje kopię tablic innych agentów, bez siebie samego.
        for i in range(self.sheepCount):
            self.sheepList[i].setLists(np.delete(self.sheepList, i), list(np.array(self.dogList)))

        for i in range(self.dogCount):
            self.dogList[i].setLists(self.sheepList, list(np.delete(self.dogList, i)))

    @property
    def action_space(self):
        dim = 3 if self.rotationMode is RotationMode.FREE else 2
        singleActionSpace = spaces.Box(-1, 1, (dim,))
        return spaces.Tuple((singleActionSpace,) * self.dogCount)

    @property
    def observation_space(self):
        singleObservationSpace = spaces.Box(-1, 1, (2, self.raysCount))
        return spaces.Tuple((singleObservationSpace,) * self.dogCount)

    def _createState(self):
        return np.ndarray(shape=(self.dogCount, 2, self.raysCount + 1), dtype=float)

    def _scatter(self):
        self.herdCentrePoint[0] = self.herdCentrePoint[1] = 0
        for sheep in self.sheepList:
            self.herdCentrePoint[0] += sheep.x
            self.herdCentrePoint[1] += sheep.y

        self.herdCentrePoint[0] /= self.sheepCount
        self.herdCentrePoint[1] /= self.sheepCount

        self.previousScatter = self.scatter
        self.scatter = 0
        for sheep in self.sheepList:
            self.scatter += (sheep.x - self.herdCentrePoint[0]).__pow__(2) + (
                sheep.y - self.herdCentrePoint[1]).__pow__(2)

    def _checkIfDone(self):
        if self.scatter < self.params.SCATTER_LEVEL:
            return True
        return False

    def _reward(self):
        self._scatter()

        returnValue = 0
        if self.scatter < self.previousScatter:
            returnValue = self.params.REWARD
        elif self.scatter <= self.params.SCATTER_LEVEL:
            returnValue = self.params.REWARD_FOR_HERDING
        else:
            returnValue = -self.params.REWARD

        return returnValue
