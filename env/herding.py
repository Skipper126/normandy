import gym
import random
from gym.envs.classic_control import rendering
from gym import spaces
import numpy as np

class SheepBehaviour:
    SIMPLE, COMPLEX = range(2)
class AgentsLayout:

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


class Agent:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.sheepList = None
        self.dogList = None
        self.transform = rendering.Transform()
        self.transform.set_translation(self.x, self.y)
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
        self.move = None
        self.setBehaviourMode(behaviour)
        self.body.set_color(181 / 255, 185 / 255, 215 / 255)

    def setBehaviourMode(self, behaviour):
        if behaviour is SheepBehaviour.SIMPLE:
            self.move = self._simpleMove
        elif behaviour is SheepBehaviour.COMPLEX:
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
        pass


class Dog(Agent):

    def __init__(self):
        Agent.__init__(self)

        self.body.set_color(185 / 255, 14 / 255, 37 / 255)
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

    defaultParams = {
        'sheep count': 7,
        'dog count': 2,
        'sheep behaviour': SheepBehaviour.SIMPLE,
        'field of view': 180,
        'rays count': 128,
        'max movement delta': 5,
        'max rotation delta': 90,
        'map height': 600,
        'map width': 1200,
        'layout': AgentsLayout.RANDOM,
        'rotation mode': RotationMode.FREE
    }

    def __init__(self, params=None):
        self.params = self.mergeParams(params)
        self.viewer = None
        self.mapHeight = self.params['map height']
        self.mapWidth = self.params['map width']
        self.sheepCount = self.params['sheep count']
        self.dogCount = self.params['dog count']
        self.maxMovementDelta = self.params['max movement delta']
        self.maxRotationDelta = self.params['max rotation delta']
        self.raysCount = self.params['rays count']
        self.sheepList = []
        self.dogList = []
        self.agentsLayoutFunc = self.params['layout']
        self.sheepBehaviour = self.params['sheep behaviour']

        self.createAgents()
        self.action_space = self.createActionSpace()
        self.observation_space = self.createObservationSpace()

    def _step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        for i, dog in enumerate(self.dogList):
            # 0 - deltaX, 1 - deltaY, 2 - deltaRotation
            dog.move(action[i][0], action[i][1], action[i][2])
        for sheep in self.sheepList:
            sheep.move()

        return self.observation(), self.reward(), self.checkIfDone(), {}

    def _reset(self):
        # Metoda statyczna klasy AgentsLayout. Wyjątkowo przyjmuje parametr self.
        self.agentsLayoutFunc(self)
        return [], 0, False, {}

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

        return self.viewer.render()

    def setAgentsLayout(self, layout):
        self.agentsLayoutFunc = layout

    def setSheepBehaviourMode(self, behaviour):
        for sheep in self.sheepList:
            sheep.setBehaviourMode(behaviour)

    def createAgents(self):
        for i in range(self.sheepCount):
            self.sheepList.append(Sheep(self.sheepBehaviour))
        for i in range(self.dogCount):
            self.dogList.append(Dog())

        # Każda owca dostaje kopię tablicy z innymi owcami oraz tablicę psów.
        for i in range(self.sheepCount):
            self.sheepList[i].setLists(np.delete(self.sheepList, i), self.dogList)

    def mergeParams(self, params):
        if params is not None:
            newParams = self.defaultParams.copy()
            newParams.update(params)
            return newParams
        else:
            return self.defaultParams

    def createActionSpace(self):
        """
        Stworzenie action space zależnego od ilości psów w środowisku.
        Action space to krotka zawierająca wejscia dla kolejnych psów. 
        Każde wejscie to MultiDiscrete action space zawierający:
        deltaX, deltaY, deltaRotation.
        """
        singleActionSpace = spaces.MultiDiscrete(
            np.array([[-self.maxMovementDelta, self.maxMovementDelta],
                      [-self.maxMovementDelta, self.maxMovementDelta],
                      [-self.maxRotationDelta, self.maxRotationDelta]]))

        return spaces.Tuple((singleActionSpace,) * self.dogCount)

    def createObservationSpace(self):
        return np.ndarray(shape=(self.dogCount, 2, self.raysCount), dtype=float)

    def checkIfDone(self):
        """
        Sprawdzanie czy zakończyć już symulację
        """
        # TODO
        return False

    def reward(self):
        """
        Obliczanie nagrody za dany ruch. Sposób obliczenia jeszcze nie ustalony.        
        """
        # TODO
        return 0

    def observation(self):
        """
        Metoda przeprowadzająca raytracing i aktualizująca tablicę observation_space.
        Wymiary tablicy observation_space:
            pierwszy(i) - indeks psa,
            drugi(0, 1) - odpowiednio:
                0 - tablica wyników raytracingu (wartości od 0 do 1),
                1 - tablica określająca obiekt trafiony przez promień:
                    -1 - owca
                    0 - nic
                    1 - pies
            trzeci(j) - wartości tablic         
        """
        for i in range(self.dogCount):
            for j in range(self.raysCount):
                # TODO
                self.observation_space[i][0][j] = random.uniform(0, 1)  # Kod przykładowy
                self.observation_space[i][1][j] = random.randint(-1, 1)
        return self.observation_space



def manualSteering():

    # Zbiór parametrów do środowiska przekazywanych do konstruktora.
    params = {
        'sheep count': 40,
        'dog count': 11,
    }
    # main






if __name__ == "__main__":
    manualSteering()

