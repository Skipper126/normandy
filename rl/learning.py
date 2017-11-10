from env.herding import Herding, EnvParams, RotationMode, AgentsLayout
from tensorforce.agents import TRPOAgent
from tensorforce.execution import Runner

from rl.multi_agent_wrapper import MultiAgentWrapper
from tensorforce import Configuration
import win32api
from rl.env_wrapper import EnvWrapper, OpenAIWrapper


params = EnvParams()
params.DOG_COUNT = 1
params.SHEEP_COUNT = 20
params.RAYS_COUNT = 128
params.RAY_LENGTH = 600
params.FIELD_OF_VIEW = 220
params.MAX_ROTATION_DELTA = 20
params.LAYOUT_FUNCTION = AgentsLayout.DOGS_OUTSIDE_CIRCLE
env = OpenAIWrapper(EnvWrapper(params), 'herding')

SAVE_DIRECTORY = './model/'
REPEAT_ACTIONS = 5
MAX_EPISODE_TIMESTEPS = 3000

agent = MultiAgentWrapper(TRPOAgent, dict(
    states_spec=env.states,
    actions_spec=env.actions,
    network_spec=[
        dict(type='dense', size=128),
        dict(type='dense', size=64)
    ],
    config=Configuration(
        batch_size=4096,
        normalize_rewards=True
    )
), params.DOG_COUNT)
# Create the runner
runner = Runner(agent=agent, environment=env, repeat_actions=REPEAT_ACTIONS)


# Callback function printing episode statistics
def episode_finished(r):
    print("Finished episode {ep} after {ts} timesteps (reward: {reward})".format(ep=r.episode, ts=r.timestep,
                                                                                 reward=r.episode_rewards[-1]))

    if r.episode >= 90 and r.episode % 100 == 0:
        r.agent.save_model(SAVE_DIRECTORY)
        print('model saved')
    # if win32api.GetAsyncKeyState(ord('P')):
    #     while True:
    #         if win32api.GetAsyncKeyState(ord('C')):
    #             break
    #         state = r.environment.reset()
    #         r.agent.reset()
    #         timestep = 0
    #         while True:
    #             if win32api.GetAsyncKeyState(ord('C')):
    #                 break
    #             action = r.agent.act(states=state)
    #             terminal = False
    #             for _ in range(REPEAT_ACTIONS):
    #                 state, terminal, reward = r.environment.execute(actions=action)
    #                 timestep += 1
    #                 r.environment.gym.render()
    #             if terminal is True or timestep == MAX_EPISODE_TIMESTEPS:
    #                 timestep = 0
    #                 break
    # if win32api.GetAsyncKeyState(ord('S')):
    #     r.agent.save_model(SAVE_DIRECTORY)
    #     print('model saved')
    # if win32api.GetAsyncKeyState(ord('L')):
    #     r.agent.load_model(SAVE_DIRECTORY)
    #     print('model restored')
    return True


runner.run(episode_finished=episode_finished, max_episode_timesteps=MAX_EPISODE_TIMESTEPS)
