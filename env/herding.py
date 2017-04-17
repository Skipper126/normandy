import gym
import random
from gym.envs.classic_control import rendering
from gym import spaces
import numpy as np


class SheepBehaviour:
    SIMPLE, COMPLEX = range(2)


class AgentsLayout:
    """
    AgentsLayout zawiera statyczne metody do rozstawienia agentów na przy starcie rundy
    """
    @staticmethod
    def _randomLayout(env):
        padding = 5
        for agent in env.dogList + env.sheepList:
            x = random.randint(agent.radius + padding, env.mapWidth - agent.radius - padding)
            y = random.randint(agent.radius + padding, env.mapHeight - agent.radius - padding)
            agent.setPos(x, y)

    @staticmethod
    def _dogsOutsideCircleLayout(env):
        # TODO
        pass

    @staticmethod
    def _dogsInsideCircleLayout(env):
        # TODO
        pass

    RANDOM = _randomLayout
    DOGS_OUTSIDE_CIRCLE = _dogsOutsideCircleLayout
    DOGS_INSIDE_CIRCLE = _dogsInsideCircleLayout


class RotationMode:
    FREE, LOCKED_ON_HERD_CENTRE = range(2)


class HerdingParams:

    def __init__(self):
        self.SHEEP_COUNT = 7
        self.DOG_COUNT = 2
        self.SHEEP_BEHAVIOUR = SheepBehaviour.SIMPLE
        self.FIELD_OF_VIEW = 180
        self.RAYS_COUNT = 128
        self.MAX_MOVEMENT_DELTA = 5
        self.MAX_ROTATION_DELTA = 90
        self.MAP_HEIGHT = 600
        self.MAP_WIDTH = 1200
        self.LAYOUT = AgentsLayout.RANDOM
        self.ROTATION_MODE = RotationMode.FREE


class Agent:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.sheepList = None
        self.dogList = None
        self.transform = rendering.Transform()
        self.radius = 15
        self.body = rendering.make_circle(self.radius, res=50)
        self.body.add_attr(self.transform)

    def getBody(self):
        return self.body

    def setPos(self, x, y):
        self.x = x
        self.y = y
        self.transform.set_translation(self.x, self.y)

    def setLists(self, sheepList, dogList):
        self.dogList = dogList
        self.sheepList = sheepList


class Sheep(Agent):

    def __init__(self, behaviour):
        Agent.__init__(self)

        self.move = self._simpleMove
        self.setBehaviourMode(behaviour)
        self.body.set_color(181 / 255, 185 / 255, 215 / 255)

    def setBehaviourMode(self, behaviour):
        if behaviour is SheepBehaviour.SIMPLE:
            self.move = self._simpleMove
        else:
            self.move = self._complexMove

    """
    _simpleMove i _complexMove to metody poruszania się owcy.
    Jedna z nich zostaje przypisana do self.move i wywoływana jako move()
    """
    def _simpleMove(self):
        # self.sheepList zawiera tylko owce różne od danej
        # TODO

        self.transform.set_translation(self.x, self.y)

    def _complexMove(self):
        # TODO

        self.transform.set_translation(self.x, self.y)
        pass


class Dog(Agent):

    """
    RAYS i TARGETS to 'stałe' używane przy indeksowaniu wymiaru tablicy observation.
    observation[RAYS] odnosi się do wektora wartości promieni,
    observation[TARGETS] odnosie się do wektora z informają o celu w jaki trafił promień
    """
    RAYS = 0
    TARGETS = 1

    def __init__(self, observationSpace, rotationMode):
        Agent.__init__(self)

        self.body.set_color(185 / 255, 14 / 255, 37 / 255)
        self.rotation = 0
        self.observation = observationSpace
        self.rotationMode = rotationMode

    def move(self, action):
        # TODO sterowanie zależne od rotacji
        self.x += action[0]
        self.y += action[1]

        if self.rotationMode is RotationMode.FREE:
            self.rotation = action[2]
        else:
            self.rotation = self._calculateRotation()

        self.transform.set_translation(self.x, self.y)
        self.transform.set_rotation(self.rotation)

    def _calculateRotation(self):
        # Obliczenie rotacji wskazującej środek ciężkości stada owiec
        # TODO
        return 0

    def updateObservation(self):
        """
        Metoda przeprowadzająca raytracing. Zmienna observation wskazuje na tablicę observation_space[i]
        środowiska, gdzie indeks 'i' oznacza danego psa.
        """
        # TODO
        # Przykład użycia:
        for i, _ in enumerate(self.observation[self.RAYS]):
            self.observation[self.RAYS][i] = random.uniform(0, 1)

        for i, _ in enumerate(self.observation[self.TARGETS]):
            self.observation[self.TARGETS][i] = random.randint(-1, 1)


class Herding(gym.Env):

    metadata = {
        'render.modes': ['human']
    }

    def __init__(self, params=None):
        self.params = HerdingParams() if params is None else params
        self.mapHeight = self.params.MAP_HEIGHT
        self.mapWidth = self.params.MAP_WIDTH
        self.sheepCount = self.params.SHEEP_COUNT
        self.dogCount = self.params.DOG_COUNT
        self.maxMovementDelta = self.params.MAX_MOVEMENT_DELTA
        self.maxRotationDelta = self.params.MAX_ROTATION_DELTA
        self.raysCount = self.params.RAYS_COUNT
        self.agentsLayoutFunc = self.params.LAYOUT
        self.sheepBehaviour = self.params.SHEEP_BEHAVIOUR
        self.rotationMode = self.params.ROTATION_MODE

        self.sheepList = []
        self.dogList = []
        self.viewer = None

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
        self.agentsLayoutFunc(self)

        for dog in self.dogList:
            dog.updateObservation()

        return self.observation_space

    def _render(self, mode='human', close=False):

        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        if self.viewer is None:
            self.viewer = rendering.Viewer(self.mapWidth, self.mapHeight)
            for agent in self.sheepList + self.dogList:
                self.viewer.add_geom(agent.getBody())

        self.viewer.render()

    def setAgentsLayout(self, layout):
        """
        Metoda ustawia funkcję to rozstawiania agentów przy wywołaniu reset()
        """
        self.agentsLayoutFunc = layout

    def setSheepBehaviourMode(self, behaviour):
        for sheep in self.sheepList:
            sheep.setBehaviourMode(behaviour)

    def _createAgents(self):
        for i in range(self.sheepCount):
            self.sheepList.append(Sheep(self.sheepBehaviour))
        for i in range(self.dogCount):
            """
            Każdy z psów otrzymuje wskaźnik na fragment observation_space dotyczący jego
            obserwacji. Przy każdym kroku będzie aktualizował swój fragment tablicy.
            """
            self.dogList.append(Dog(self.observation_space[i], self.params.ROTATION_MODE))

        # Każda owca dostaje kopię tablicy z innymi owcami oraz tablicę psów.
        for i in range(self.sheepCount):
            self.sheepList[i].setLists(np.delete(self.sheepList, i), self.dogList)

    def _createActionSpace(self):
        """
        Stworzenie action space zależnego od ilości psów w środowisku.
        Action space to krotka zawierająca wejscia dla kolejnych psów. 
        Każde wejscie to MultiDiscrete action space zawierający:
        deltaX, deltaY i zależnie od parametrów deltaRotation.
        """
        singleActionSpace = None

        if self.rotationMode is RotationMode.FREE:
            singleActionSpace = spaces.MultiDiscrete(
                np.array([[-self.maxMovementDelta, self.maxMovementDelta],
                          [-self.maxMovementDelta, self.maxMovementDelta],
                          [-self.maxRotationDelta, self.maxRotationDelta]]))

        elif self.rotationMode is RotationMode.LOCKED_ON_HERD_CENTRE:
            singleActionSpace = spaces.MultiDiscrete(
                np.array([[-self.maxMovementDelta, self.maxMovementDelta],
                          [-self.maxMovementDelta, self.maxMovementDelta]]))

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
        return 0


# KOD WYKONYWANY PRZY BEZPOŚREDNIM URUCHAMIANIU PLIKU herding.py
def manualSteering():
    # main
    # kod tutaj jest tylko dla przykładu i jest w pełni do nadpisania
    import time
    # Zbiór parametrów do środowiska przekazywanych do konstruktora.
    params = HerdingParams()
    params.DOG_COUNT = 4
    params.MAX_MOVEMENT_DELTA = 10
    env = Herding(params)
    env.reset()
    for _ in range(100):
        env.step(env.action_space.sample())
        env.render()
        time.sleep(0.05)

    env.close()



if __name__ == "__main__":
    manualSteering()

