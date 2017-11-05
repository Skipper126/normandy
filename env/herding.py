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
        """
        Stworzenie action space zależnego od ilości psów w środowisku.
        Action space to krotka zawierająca wejscia dla kolejnych psów. 
        Każde wejscie to MultiDiscrete action space zawierający:
        deltaX, deltaY i zależnie od parametrów deltaRotation.
        """
        dim = 3 if self.rotationMode is RotationMode.FREE else 2
        singleActionSpace = spaces.Box(-1, 1, (dim,))
        return singleActionSpace

    @property
    def observation_space(self):
        return spaces.Box(-1, 1, (self.raysCount * 2,))

    def _createState(self):
        """
        Wymiary tablicy observation_space:
            pierwszy - tablica obserwacji dla każdego z psów
            drugi - tablica wektorów obserwacji
                0 - wektor wyników raytracingu (wartości od 0 do 1),
                1 - wektor określający obiekt trafiony przez promień:
                    -1 - owca
                    0 - nic
                    1 - pies
            trzeci - poszczególne wartości wektora
        """
        return np.ndarray(shape=(self.dogCount, 2, self.raysCount), dtype=float)

    def _checkIfDone(self):
        """
        Sprawdzanie czy zakończyć już symulację
        """
        # TODO
        self.epoch += 1
        if self.epoch == self.params.EPOCH or self.scatter < self.params.SCATTER_LEVEL:
            return True

        return False

    def _scatter(self):
        # do sprawdzenia
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

    def _reward(self):
        """
        Obliczanie nagrody za dany ruch. Sposób obliczenia jeszcze nie ustalony,
        najprawdopodobniej będzie to suma kwadratów odległości owiec od ich środka ciężkości.
        Wyliczenie scatter - rozproszenie
        """
        self._scatter()

        #do sprawdzenia
        #self.rewardValue = EULER.__pow__(((self.previousScatter - self.scatter).__pow__(2) / 2).__neg__())
        self.rewardValue = self.previousScatter - self.scatter
        if self.scatter < self.previousScatter:
            self.rewardValue.__neg__()
        if self.scatter < self.params.SCATTER_LEVEL:
            self.rewardValue = self.params.REWARD_FOR_HERDING

        #print(self.rewardValue, self.scatter, self.constansScatterCounter)

        return self.rewardValue
