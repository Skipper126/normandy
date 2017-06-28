from .agent import Agent
from ..constants import *
import math


class Dog(Agent):

    """
    RAYS i TARGETS to stałe używane przy indeksowaniu wymiaru tablicy observation.
    observation[RAYS] odnosi się do wektora wartości promieni,
    observation[TARGETS] odnosie się do wektora z informają o celu w jaki trafił promień
    """
    RAYS = 0
    TARGETS = 1

    def __init__(self, observationSpace, envParams: EnvParams):
        super().__init__(envParams)

        self.rotation = 0
        self.observation = observationSpace
        self.rotationMode = self.params.ROTATION_MODE
        self.rayRadian = []
        for i in range(self.params.RAYS_COUNT):
            self.rayRadian.append((PI + ((180 - self.params.FIELD_OF_VIEW) / 360) * PI + (self.params.FIELD_OF_VIEW / (self.params.RAYS_COUNT - 1)) * DEG2RAD * i) % TWOPI)
        if self.rayRadian[0] > self.rayRadian[self.params.RAYS_COUNT - 1]:
            self.wideView = True
        else:
            self.wideView = False

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
            self.rotation = self.rotation % TWOPI
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

    def clearObservation(self):
        for i, _ in enumerate(self.observation[self.RAYS]):
            self.observation[self.RAYS][i] = 1
            self.observation[self.TARGETS][i] = 0

    def getDistanceFromAgent(self, agent):
        return pow(pow((self.x - agent.x), 2) + pow((self.y - agent.y), 2), 0.5)

    def calculateAngle(self, agent):
        tempAngle = math.atan2(self.y - agent.y, self.x - agent.x) - self.rotation
        while tempAngle < 0:
            tempAngle += TWOPI
        return tempAngle

    def calculateDelta(self, rayTan, agent):
        return pow((2 * (self.x - agent.x)) + (2 * rayTan * (self.y - agent.y)), 2) - (4 * (1 + pow(rayTan, 2)) * (-1 * pow(self.radius, 2) + pow(self.x - agent.x, 2) + pow(self.y - agent.y, 2)))

    def calculateStraightToCircleDistance(self, agent, index):
        return abs(-1 * math.tan(self.rotation - self.rayRadian[index]) * (self.x - agent.x) + self.y - agent.y) / pow(pow(math.tan(self.rotation - self.rayRadian[index]), 2) + 1, 0.5)

    def isInSight(self, tempAngle):
        if self.wideView:
            if not self.rayRadian[self.params.RAYS_COUNT-1] < tempAngle < self.rayRadian[0]:
                return True
        else:
            if self.rayRadian[0] < tempAngle < self.rayRadian[self.params.RAYS_COUNT-1]:
                return True
        return False

    def setDistanceAndColor(self, index, agent):
        rayTan = math.tan(self.rotation - self.rayRadian[index])
        delta = self.calculateDelta(rayTan, agent)
        x1 = (((2 * (self.x - agent.x)) + (2 * rayTan * (self.y - agent.y))) - math.pow(delta, 0.5)) / (2 * (1 + pow(rayTan, 2)))
        y1 = rayTan * x1
        x2 = (((2 * (self.x - agent.x)) + (2 * rayTan * (self.y - agent.y))) + math.pow(delta, 0.5)) / (2 * (1 + pow(rayTan, 2)))
        y2 = rayTan * x2
        distance1 = pow(pow(x1, 2) + pow(y1, 2), 0.5)
        distance2 = pow(pow(x2, 2) + pow(y2, 2), 0.5)
        if distance1 < distance2:
            distance = distance1 - self.radius
        else:
            distance = distance2 - self.radius
        self.observation[self.RAYS][index] = distance / self.params.RAY_LENGTH
        self.observation[self.TARGETS][index] = 1 if type(agent) is Dog else -1

    def iterateRays(self, distance, agent, index, iterator):
        while 0 <= index <= self.params.RAYS_COUNT - 1:
            circleDistance = self.calculateStraightToCircleDistance(agent, index)
            if circleDistance <= self.radius:
                if (distance - (2 * self.radius)) / self.params.RAY_LENGTH < self.observation[self.RAYS][index]:
                    self.setDistanceAndColor(index, agent)
            else:
                break
            index += iterator

    def colorRays(self, tempAngle, distance, agent):
        if tempAngle < self.rayRadian[0]:
            tempAngle += TWOPI
        left = self.params.RAYS_COUNT - 2 - int((tempAngle - self.rayRadian[0]) / ((self.params.FIELD_OF_VIEW / (self.params.RAYS_COUNT - 1)) * DEG2RAD))
        right = left + 1
        # color left rays
        self.iterateRays(distance, agent, left, -1)
        # color right rays
        self.iterateRays(distance, agent, right, 1)

    def updateObservation(self):
        """
        Metoda przeprowadzająca raytracing. Zmienna observation wskazuje na tablicę observation_space[i]
        środowiska, gdzie indeks 'i' oznacza danego psa.
        """
        self.clearObservation()
        for agent in self.sheepList + self.dogList:
            distance = self.getDistanceFromAgent(agent)
            if distance - (2 * self.radius) < self.params.RAY_LENGTH:
                tempAngle = self.calculateAngle(agent)
                if self.isInSight(tempAngle):
                    self.colorRays(tempAngle, distance, agent)