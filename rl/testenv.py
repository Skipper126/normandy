from env.herding import Herding, EnvParams, RotationMode, AgentsLayout
import numpy as np
from gym import spaces


class TestEnv(Herding):

    def _step(self, action):
        self.dogList[0].move(action)

        for sheep in self.sheepList:
            sheep.move()

        self.dogList[0].updateObservation()

        return np.append(self.state[0][0], self.state[0][1]), self._reward(), self._checkIfDone(), {}

    def _reset(self):
        self.setUpAgents(self)
        self.dogList[0].updateObservation()

        return np.append(self.state[0][0], self.state[0][1])

    def _checkIfDone(self):
        if self.scatter < self.params.SCATTER_LEVEL:
            return True

        if self.dogList[0].y > self.mapHeight + 100 or self.dogList[0].y < -100 or \
                self.dogList[0].x > self.mapWidth + 100 or self.dogList[0].x < -100:
                return True
        if np.abs(self.dogList[0].y - self.herdCentrePoint[1]) < 5 and \
                np.abs(self.dogList[0].x - self.herdCentrePoint[0]) < 5:
                return True
        return False

    def _reward(self):
        self._scatter()
       #self.rewardValue = EULER.__pow__(((self.previousScatter - self.scatter).__pow__(2) / 2).__neg__())
        self.rewardValue = self.previousScatter - self.scatter
        if self.scatter < self.previousScatter:
            self.rewardValue.__neg__()


        #print(self.rewardValue, self.scatter, self.constansScatterCounter)
        returnValue = -0.5 if self.rewardValue <= 0 else 1
        if self.dogList[0].y > self.mapHeight + 100 or self.dogList[0].y < -100 or \
                self.dogList[0].x > self.mapWidth + 100 or self.dogList[0].x < -100:
            returnValue = -100
        if np.abs(self.dogList[0].y - self.herdCentrePoint[1]) < 5 and \
                        np.abs(self.dogList[0].x - self.herdCentrePoint[0]) < 5:
            returnValue = -100
        if self.scatter < self.params.SCATTER_LEVEL:
            self.rewardValue = self.params.REWARD_FOR_HERDING
            returnValue = 100

        return returnValue

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

    def _close(self):
        pass