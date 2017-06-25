from ..constants import EnvParams


class Agent:

    def __init__(self, envParams: EnvParams):
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
