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

        for agent in self.sheepList + self.dogList:
            distance = pow(pow((self.x - agent.x), 2) + pow((self.y - agent.y), 2), 0.5)
            if distance < self.params.RAY_LENGTH:
                for i in range(self.params.RAYS_COUNT):
                    # if ((self.y - agent.y) > 0 and self.rotation - self.rayRadian[i] > PI) or ((self.y - agent.y) < 0 and self.rotation - self.rayRadian[i] <= PI):
                    circleDistance = abs(-1*math.tan(self.rotation - self.rayRadian[i]) * (self.x - agent.x) + self.y - agent.y) / pow(pow(math.tan(self.rotation - self.rayRadian[i]), 2) + 1, 0.5)
                    if circleDistance <= self.radius:
                        self.observation[self.RAYS][i] = distance / self.params.RAY_LENGTH
                        self.observation[self.TARGETS][i] = 1 if type(agent) is Dog else -1
