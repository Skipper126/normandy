import gym
import random
from gym.envs.classic_control import rendering
from gym import spaces
import numpy as np


class Agent:

    def __init__(self):
        self.x = random.randint(20, 1260)
        self.y = random.randint(20, 780)
        self.sheepList = None
        self.wolfList = None
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

    def setLists(self, sheepList, wolfList):
        self.wolfList = wolfList
        self.sheepList = sheepList


class Sheep(Agent):

    def __init__(self):
        Agent.__init__(self)

        self.body.set_color(181 / 255, 185 / 255, 215 / 255)

    def move(self):
        # TODO


        self.transform.set_translation(self.x, self.y)


class Wolf(Agent):

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
        'wolf count': 2,
        'sheep behaviour': 'simple',
        'field of view': 180,
        'rays count': 128,
        'max movement delta': 5,
        'max rotation delta': 90
    }

    layout = {
        'random': 0,
        'circle': 1,
        'wolfs inside circle': 2
    }

    def __init__(self, params=None):
        self.params = self.defaultParams if params is None else params

        self.viewer = None
        self.sheepCount = self.params['sheep count']
        self.wolfCount = self.params['wolf count']
        self.maxMovementDelta = self.params['max movement delta']
        self.maxRotationDelta = self.params['max rotation delta']
        self.raysCount = self.params['rays count']
        self.sheepList = []
        self.wolfList = []
        self.agentsLayout = self.layout['random']

        self.createAgents()
        self.action_space = self.createActionSpace()
        self.observation_space = self.createObservationSpace()

    def _step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        for i, wolf in enumerate(self.wolfList):
            wolf.move(action[i][0], action[i][1], action[i][2])
        for sheep in self.sheepList:
            sheep.move()

        return self.observation(), self.reward(), self.checkIfDone(), {}

    def _reset(self):
        return [], 0, False, {}

    def _render(self, mode='human', close=False):

        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        if self.viewer is None:
            screenWidth = 1280
            screenHeight = 800
            self.viewer = rendering.Viewer(screenWidth, screenHeight)

            for agent in self.sheepList + self.wolfList:
                self.viewer.add_geom(agent.getBody())

        return self.viewer.render()

    def setAgentsLayout(self, layout):
        self.agentsLayout = layout


        # TODO

    def createAgents(self):
        for i in range(self.sheepCount):
            self.sheepList.append(Sheep())
        for i in range(self.wolfCount):
            self.wolfList.append(Wolf())

        # Każda owca dostaje kopię tablicy z innymi owcami oraz tablicę wilków.
        for i in range(self.sheepCount):
            self.sheepList[i].setLists(np.delete(self.sheepList, i), self.wolfList)

    def createActionSpace(self):
        """
        Stworzenie action space zależnego od ilości wilków w środowisku.
        Action space to krotka zawierająca wejscia dla kolejnych wilków. 
        Każde wejscie to MultiDiscrete action space zawierający:
        deltaX, deltaY, deltaRotation.
        """
        singleActionSpace = spaces.MultiDiscrete(
            np.array([[-self.maxMovementDelta, self.maxMovementDelta],
            [-self.maxMovementDelta, self.maxMovementDelta],
            [-self.maxRotationDelta, self.maxRotationDelta]]))

        return spaces.Tuple((singleActionSpace,)*self.wolfCount)

    def createObservationSpace(self):
        return np.ndarray(shape=(self.wolfCount, 2, self.raysCount), dtype=float)

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
            pierwszy(i) - indeks wilka,
            drugi(0, 1) - odpowiednio:
                0 - tablica wyników raytracingu (wartości od 0 do 1),
                1 - tablica określająca obiekt trafiony przez promień:
                    -1 - owca
                    0 - nic
                    1 - wilk
            trzeci(j) - wartości tablic         
        """
        for i in range(self.wolfCount):
            for j in range(self.raysCount):
                # TODO
                self.observation_space[i][0][j] = random.uniform(0, 1)
                self.observation_space[i][1][j] = random.randint(-1, 1)
        return self.observation_space


def manualSteering():

    # Zbiór parametrów do środowiska przekazywanych do konstruktora.
    params = {
        'sheep count': 4,
        'wolf count': 1,
        'sheep behaviour': 'simple',
        'field of view': 180,
        'rays count': 128,
        'max movement delta': 5,
        'max rotation delta': 90
    }

    env = Herding()
    env.reset()
    env.render()
    input()





if __name__ == "__main__":
    manualSteering()


