from env.herding import Herding
from env.constants import EnvParams
from pyglet.window import key
import numpy as np
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
params = EnvParams()
params.DOG_COUNT = 3
params.SHEEP_COUNT = 50
params.RAYS_COUNT = 180
params.FIELD_OF_VIEW = 180
env = Herding(params)
env.reset()
env.render()
env.viewer.viewer.window.on_key_press = key_press
env.viewer.viewer.window.on_key_release = key_release
while not closeEnv[0]:
    env.step((np.array([vector[0], vector[1], vector[2]]), np.array([0, 0, 0]), np.array([0, 0, 0])))
    env.render()
    time.sleep(0.005)

env.close()
