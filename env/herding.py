import gym
import random
from gym.envs.classic_control import rendering
from gym import spaces
import numpy as np

TWOPI = 2 * 3.14159265359

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
        self.AGENT_RADIUS = 15
        self.SHEEP_BEHAVIOUR = SheepBehaviour.SIMPLE
        self.FIELD_OF_VIEW = 180
        self.RAYS_COUNT = 128
        self.MAX_MOVEMENT_DELTA = 5
        self.MAX_ROTATION_DELTA = 90
        self.MAP_HEIGHT = 800
        self.MAP_WIDTH = 1200
        self.LAYOUT = AgentsLayout.RANDOM
        self.ROTATION_MODE = RotationMode.FREE


class HerdingRenderer:

    class _Geom:

        def __init__(self, envObject, envParams):
            self.params = envParams
            self.object = envObject
            self.geomPartList = []
            self.geomPartTransformList = []
            self._createBody()
            self._addTransformToAllParts()

        def _createBody(self):
            raise NotImplementedError

        def _addTransformToAllParts(self):
            for part in self.geomPartList:
                transform = rendering.Transform()
                self.geomPartTransformList.append(transform)
                part.add_attr(transform)

        def update(self):
            raise NotImplementedError

    class _Sheep(_Geom):

        BODY = 0

        def _createBody(self):
            body = rendering.make_circle(self.object.radius, res=50)
            body.set_color(181 / 255, 185 / 255, 215 / 255)
            self.geomPartList.append(body)

        def update(self):
            tr = self.geomPartTransformList
            tr[self.BODY].set_translation(self.object.x, self.object.y)

    class _Dog(_Geom):

        BODY = 0
        RAY = 1
        # TODO rysowanie promieni raytracingu

        def _createBody(self):
            body = rendering.make_circle(self.object.radius, res=50)
            body.set_color(185 / 255, 14 / 255, 37 / 255)
            self.geomPartList.append(body)

            # line = rendering.Line((0, 0), (50, 0))
            # self.geomPartList.append(line)

        def update(self):
            tr = self.geomPartTransformList
            tr[self.BODY].set_translation(self.object.x, self.object.y)
            tr[self.BODY].set_rotation(self.object.rotation)


    def __init__(self, sheepList, dogList, envParams):
        self.params = envParams
        self.mapWidth = self.params.MAP_WIDTH
        self.mapHeight = self.params.MAP_HEIGHT
        self.dogList = dogList
        self.sheepList = sheepList
        self.geomList = []
        self.viewer = rendering.Viewer(self.mapWidth, self.mapHeight)
        self._initRenderObjects()

    def _initRenderObjects(self):
        for sheep in self.sheepList:
            self.geomList.append(self._Sheep(sheep, self.params))

        for dog in self.dogList:
            self.geomList.append(self._Dog(dog, self.params))

        for geom in self.geomList:
            self.viewer.geoms.extend(geom.geomPartList)

    def render(self):
        for geom in self.geomList:
            geom.update()

        self.viewer.render()

    def close(self):
        self.viewer.close()


class Agent:

    def __init__(self, envParams):
        self.x = 0
        self.y = 0
        self.sheepList = None
        self.dogList = None
        self.params = envParams
        self.radius = self.params.AGENT_RADIUS

    def setPos(self, x, y):
        self.x = x
        self.y = y

    def setLists(self, sheepList, dogList):
        self.dogList = dogList
        self.sheepList = sheepList


class Sheep(Agent):

    def __init__(self, envParams):
        super().__init__(envParams)

        self.move = self._simpleMove
        self.setBehaviourMode(self.params.SHEEP_BEHAVIOUR)

    def setBehaviourMode(self, behaviour):
        if behaviour is SheepBehaviour.SIMPLE:
            self.move = self._simpleMove
        else:
            self.move = self._complexMove

    """
    _simpleMove i _complexMove to metody poruszania się owcy.
    Jedna z nich zostaje przypisana do self.move i wywoływana jako move().
    self.sheepList zawiera tylko owce różne od danej
    """
    def _simpleMove(self):
        # TODO
        pass

    def _complexMove(self):
        # TODO
        pass


class Dog(Agent):

    """
    RAYS i TARGETS to stałe używane przy indeksowaniu wymiaru tablicy observation.
    observation[RAYS] odnosi się do wektora wartości promieni,
    observation[TARGETS] odnosie się do wektora z informają o celu w jaki trafił promień
    """
    RAYS = 0
    TARGETS = 1

    def __init__(self, observationSpace, envParams):
        super().__init__(envParams)

        self.rotation = 0
        self.observation = observationSpace
        self.rotationMode = self.params.ROTATION_MODE

    def move(self, action):
        # TODO sterowanie zależne od rotacji
        deltaX = action[0] * self.params.MAX_MOVEMENT_DELTA
        deltaY = action[1] * self.params.MAX_MOVEMENT_DELTA
        self.x += deltaX
        self.y += deltaY

        """
        Rotacja jest w radianach (0, 2 * PI), action[2] jest od (-1, 1),
        MAX_ROTATION_DELTA (0, 360)
        """
        #
        if self.rotationMode is RotationMode.FREE:
            self.rotation = action[2] * (self.params.MAX_ROTATION_DELTA / 360) * TWOPI
        else:
            self.rotation = self._calculateRotation()

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

        """
        Wszystko, co dotyczy wyświetlania środowiska, jest tworzone przy pierwszym
        wywołaniu metody render().
        """
        if self.viewer is None:
            self.viewer = HerdingRenderer(self.sheepList, self.dogList, self.params)

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
            self.sheepList.append(Sheep(self.params))
        for i in range(self.dogCount):
            """
            Każdy z psów otrzymuje wskaźnik na fragment observation_space dotyczący jego
            obserwacji. Przy każdym kroku będzie aktualizował swój fragment tablicy.
            """
            self.dogList.append(Dog(self.observation_space[i], self.params))

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

