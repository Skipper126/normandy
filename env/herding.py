import gym
import random
from gym.envs.classic_control import rendering
from gym import spaces
from pyglet.window import key
import numpy as np
import math

PI = 3.14159265359
TWOPI = 2 * PI
DEG2RAD = 0.01745329252

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
        self.RAY_LENGTH = 300
        self.MAX_MOVEMENT_DELTA = 5
        self.MAX_ROTATION_DELTA = 90
        self.MAP_HEIGHT = 800
        self.MAP_WIDTH = 1200
        self.LAYOUT_FUNCTION = AgentsLayout.RANDOM
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
        COLOR = {
            -1: (1, 0, 0),
            0: (0, 0, 0),
            1: (0, 1, 0)
        }

        def _createBody(self):

            body = rendering.make_circle(self.object.radius, res=50)
            body.set_color(185 / 255, 14 / 255, 37 / 255)
            self.geomPartList.append(body)

            for _ in range(self.params.RAYS_COUNT):
                line = rendering.Line((0, 0), (self.params.RAY_LENGTH, 0))
                self.geomPartList.append(line)

        def update(self):
            tr = self.geomPartTransformList
            tr[self.BODY].set_translation(self.object.x, self.object.y)
            for i in range(self.params.RAYS_COUNT):
                tr[self.RAY + i].set_scale(self.object.observation[0][i], 0)
                self.geomPartList[self.RAY + i].set_color(*self.COLOR[self.object.observation[1][i]])
                # RAYS_COUNT nie może być równy 1
                rot = self.object.rotation - self.object.rayRadian[i]
                # ((180 - self.params.FIELD_OF_VIEW) / 360) * PI + PI - (self.params.FIELD_OF_VIEW / (self.params.RAYS_COUNT - 1)) * DEG2RAD * i
                # PI - (self.params.FIELD_OF_VIEW / (self.params.RAYS_COUNT - 1)) * DEG2RAD * i <--- z tego zrobic tablice
                tr[self.RAY + i].set_rotation(rot)
                x = math.cos(rot) * self.object.radius
                y = math.sin(rot) * self.object.radius
                tr[self.RAY + i].set_translation(self.object.x + x, self.object.y + y)


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
        deltaX = 0
        deltaY = 0
        for dog in self.dogList:
            distance = pow(pow((self.x - dog.x), 2) + pow((self.y - dog.y), 2), 0.5)
            if distance < 100:
                if distance < 50:
                    distance = 50
                deltaX += ((self.x - dog.x) / distance) * (100 - distance)
                deltaY += ((self.y - dog.y) / distance) * (100 - distance)

        if deltaX > 50 or deltaY > 50:
            if deltaX > deltaY:
                deltaY = deltaY / deltaX * 50
                deltaX = 50
            else:
                deltaX = deltaX / deltaY * 50
                deltaY = 50

        deltaX = deltaX / 50 * self.params.MAX_MOVEMENT_DELTA
        deltaY = deltaY / 50 * self.params.MAX_MOVEMENT_DELTA
        self.x += deltaX
        self.y += deltaY


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
        self.rayRadian = []
        for i in range(self.params.RAYS_COUNT):
            self.rayRadian.append(PI + ((180 - self.params.FIELD_OF_VIEW) / 360) * PI + (self.params.FIELD_OF_VIEW / (self.params.RAYS_COUNT - 1)) * DEG2RAD * i)
        for i, _ in enumerate(self.observation[self.RAYS]):
            self.observation[self.RAYS][i] = 1
            self.observation[self.TARGETS][i] = 0


    def move(self, action):
        deltaX = action[0] * self.params.MAX_MOVEMENT_DELTA
        deltaY = action[1] * self.params.MAX_MOVEMENT_DELTA

        vecLength = math.sqrt(deltaX*deltaX + deltaY * deltaY)
        if vecLength > self.params.MAX_MOVEMENT_DELTA:
            norm = self.params.MAX_MOVEMENT_DELTA / vecLength
            deltaX *= norm
            deltaY *= norm

        """
        Rotacja jest w radianach (0, 2 * PI), action[2] jest od (-1, 1),
        MAX_ROTATION_DELTA (0, 360)
        """
        if self.rotationMode is RotationMode.FREE:
            self.rotation += action[2] * self.params.MAX_ROTATION_DELTA * DEG2RAD
        else:
            self.rotation = self._calculateRotation()

        cosRotation = math.cos(self.rotation)
        sinRotation = math.sin(self.rotation)
        self.x += deltaX * cosRotation + deltaY * sinRotation
        self.y += deltaY * -cosRotation + deltaX * sinRotation

    def _calculateRotation(self):
        # Obliczenie rotacji wskazującej środek ciężkości stada owiec
        # TODO
        return 0

    def updateObservation(self):
        """
        Metoda przeprowadzająca raytracing. Zmienna observation wskazuje na tablicę observation_space[i]
        środowiska, gdzie indeks 'i' oznacza danego psa.
        """
        """
        Odpowienie ustawienie x, y, rot promienia na podstawie objX, objY, objRot:
        rot: objRot + PI + (kąt widzenia w stopniach / ilość promieni) * DEG2RAD * i
        x: objX + sin(rot) * radius
        y: objY + cos(rot) * radius
        """
        # TODO
        # Przykład użycia:
        for i, _ in enumerate(self.observation[self.RAYS]):
            self.observation[self.RAYS][i] = 1
            self.observation[self.TARGETS][i] = 0
        for dog in self.dogList:
            distance = pow(pow((self.x - dog.x), 2) + pow((self.y - dog.y), 2), 0.5)
            if distance < self.params.RAY_LENGTH:
                for i in range(self.params.RAYS_COUNT):
                    # if ((self.y - dog.y) > 0 and self.rotation - self.rayRadian[i] > PI) or ((self.y - dog.y) < 0 and self.rotation - self.rayRadian[i] <= PI):
                    circleDistance = abs((-1 * math.tan(self.rotation - self.rayRadian[i])) * (self.x - dog.x) + self.y - dog.y) / pow(pow(math.tan(self.rotation - self.rayRadian[i]), 2) + 1, 0.5)
                    if circleDistance <= self.radius:
                        self.observation[self.RAYS][i] = distance / self.params.RAY_LENGTH
                        self.observation[self.TARGETS][i] = 1
        for sheep in self.sheepList:
            distance = pow(pow((self.x - sheep.x), 2) + pow((self.y - sheep.y), 2), 0.5)
            if distance < self.params.RAY_LENGTH:
                for i in range(self.params.RAYS_COUNT):
                    # if ((self.y - sheep.y) > 0 and self.rotation - self.rayRadian[i] > PI) or ((self.y - sheep.y) < 0 and self.rotation - self.rayRadian[i] <= PI):
                    circleDistance = abs(-1*math.tan(self.rotation - self.rayRadian[i]) * (self.x - sheep.x) + self.y - sheep.y) / pow(pow(math.tan(self.rotation - self.rayRadian[i]), 2) + 1, 0.5)
                    if circleDistance <= self.radius:
                        self.observation[self.RAYS][i] = distance / self.params.RAY_LENGTH
                        self.observation[self.TARGETS][i] = -1


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
        self.setUpAgents = self.params.LAYOUT_FUNCTION
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
            self.viewer = HerdingRenderer(self.sheepList, self.dogList, self.params)

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
            self.sheepList[i].setLists(np.delete(self.sheepList, i), self.dogList)

        for i in range(self.dogCount):
            self.dogList[i].setLists(self.sheepList, np.delete(self.dogList, i))

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
    import time
    vector = [0, 0, 0]
    closeEnv = [False]

    def key_press(k, mod):
        if k == key.LEFT:
            vector[0] = -1
        elif k == key.RIGHT:
            vector[0] = 1
        elif k == key.UP:
            vector[1] = -1
        elif k == key.DOWN:
            vector[1] = 1
        elif k == key.COMMA:
            vector[2] = 0.1
        elif k == key.PERIOD:
            vector[2] = -0.1
        elif k == key.ESCAPE:
            closeEnv[0] = True

    def key_release(k, mod):
        if k == key.LEFT:
            vector[0] = 0
        elif k == key.RIGHT:
            vector[0] = 0
        elif k == key.UP:
            vector[1] = 0
        elif k == key.DOWN:
            vector[1] = 0
        elif k == key.COMMA:
            vector[2] = 0
        elif k == key.PERIOD:
            vector[2] = 0

    # Zbiór parametrów do środowiska przekazywanych do konstruktora.
    params = HerdingParams()
    params.DOG_COUNT = 1
    params.SHEEP_COUNT = 5
    params.RAYS_COUNT = 20
    params.FIELD_OF_VIEW = 120
    env = Herding(params)
    env.reset()
    env.render()
    env.viewer.viewer.window.on_key_press = key_press
    env.viewer.viewer.window.on_key_release = key_release
    while not closeEnv[0]:
        env.step((np.array([vector[0], vector[1], vector[2]]),))
        env.render()
        time.sleep(0.005)

    env.close()

if __name__ == "__main__":
    manualSteering()

