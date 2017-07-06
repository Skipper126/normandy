import gym
from gym import spaces
import numpy as np
from .constants import *
from .agents.dog import Dog
from .agents.sheep import Sheep


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

        self.action_space = self._createActionSpace()
        self.observation_space = self._createObservationSpace()
        self._createAgents()

    def _step(self, action):
        # assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        for i, dog in enumerate(self.dogList):
            # action[i] to deltaX, deltaY oraz opcjonalnie deltaRotation
            dog.move(action[i])

        for sheep in self.sheepList:
            sheep.move()

        for dog in self.dogList:
            # updateObservation() aktualizuje tablicę observation_space
            dog.updateObservation()

        return self.observation_space, self._reward(), self._checkIfDone(), {}

    def _reset(self):
        # Metoda statyczna klasy AgentsLayout. Wyjątkowo przyjmuje parametr self.
        self.setUpAgents(self)

        for dog in self.dogList:
            dog.updateObservation()

        return self.observation_space

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
            self.dogList.append(Dog(self.observation_space[i], self.params))

        # Każdy agent dostaje kopię tablic innych agentów, bez siebie samego.
        for i in range(self.sheepCount):
            self.sheepList[i].setLists(np.delete(self.sheepList, i), list(np.array(self.dogList)))

        for i in range(self.dogCount):
            self.dogList[i].setLists(self.sheepList, list(np.delete(self.dogList, i)))

    def _createActionSpace(self):
        """
        Stworzenie action space zależnego od ilości psów w środowisku.
        Action space to krotka zawierająca wejscia dla kolejnych psów. 
        Każde wejscie to MultiDiscrete action space zawierający:
        deltaX, deltaY i zależnie od parametrów deltaRotation.
        """

        dim = 3 if self.rotationMode is RotationMode.FREE else 2
        singleActionSpace = spaces.Box(-1, 1, (dim, 1))

        return spaces.Tuple((singleActionSpace,) * self.dogCount)

    def _createObservationSpace(self):
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
        return False

    def _reward(self):
        """
        Obliczanie nagrody za dany ruch. Sposób obliczenia jeszcze nie ustalony,
        najprawdopodobniej będzie to suma kwadratów odległości owiec od ich środka ciężkości.
        """
        # TODO
        self.herdCentrePoint[0] = self.herdCentrePoint[1] = 0
        for sheep in self.sheepList:
            self.herdCentrePoint[0] += sheep.x
            self.herdCentrePoint[1] += sheep.y

        self.herdCentrePoint[0] /= self.sheepCount
        self.herdCentrePoint[1] /= self.sheepCount

        return 0



