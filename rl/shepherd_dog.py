from env.herding import Herding, EnvParams, RotationMode, AgentsLayout
from tensorforce.agents import TRPOAgent
from tensorforce.execution import Runner
from tensorforce.contrib.openai_gym import OpenAIGym
from rl.multi_agent_wrapper import MultiAgentWrapper
from tensorforce import Configuration
import win32api
from rl.testenv import TestEnv

SAVE_DIRECTORY = './model/'


class OpenAIWrapper(OpenAIGym):

    def __init__(self, env, gym_id):
        self.gym_id = gym_id
        self.gym = env


params = EnvParams()
params.DOG_COUNT = 1
params.SHEEP_COUNT = 40
params.RAYS_COUNT = 128
params.RAY_LENGTH = 600
params.FIELD_OF_VIEW = 270
params.MAX_ROTATION_DELTA = 20
params.LAYOUT_FUNCTION = AgentsLayout.RANDOM
env = OpenAIWrapper(TestEnv(params), 'herding')

agent = MultiAgentWrapper(TRPOAgent, dict(
    states_spec=env.states,
    actions_spec=env.actions,
    network_spec=[
        dict(type='dense', size=300),
        dict(type='dense', size=100)
    ],
    config=Configuration(
        batch_size=4096,
        normalize_rewards=True
    )
), params.DOG_COUNT)
# Create the runner
runner = Runner(agent=agent, environment=env)


# Callback function printing episode statistics
def episode_finished(r):
    print("Finished episode {ep} after {ts} timesteps (reward: {reward})".format(ep=r.episode, ts=r.timestep,
                                                                                 reward=r.episode_rewards[-1]))
    if win32api.GetAsyncKeyState(ord('P')):
        while True:
            if win32api.GetAsyncKeyState(ord('C')):
                break
            state = r.environment.reset()
            r.agent.reset()
            while True:
                if win32api.GetAsyncKeyState(ord('C')):
                    break
                action = r.agent.act(states=state)
                state, terminal, reward = r.environment.execute(actions=action)
                r.environment.gym.render()
                if terminal is True:
                    break
    if win32api.GetAsyncKeyState(ord('S')):
        r.agent.save_model(SAVE_DIRECTORY)
        print('model saved')
    if win32api.GetAsyncKeyState(ord('L')):
        r.agent.load_model(SAVE_DIRECTORY)
        print('model restored')
    return True


# Start learning
runner.run(episode_finished=episode_finished, max_episode_timesteps=500)
