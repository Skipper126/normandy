from rl.learning import *
from env.constants import AgentsLayout
import atexit

rl = None

def exit_handler():
    global rl
    rl.save_model()

atexit.register(exit_handler)

rl = Learning(
    terminal_reward=-500,
    sheep_count=20
)
rl.load_model()
rl.learn()
rl = Learning(
    terminal_reward=0,
    sheep_count=30,
    max_episode_timesteps=5000,
    layout=AgentsLayout.RANDOM
)
rl.load_model()
rl.learn()
rl = Learning(
    terminal_reward=9999999,
    sheep_count=30,
    max_episode_timesteps=8000,
    layout=AgentsLayout.RANDOM,
    dog_count=3
)
rl.load_model()
rl.learn()


