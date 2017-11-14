import sys
import os
from env.herding import EnvParams, AgentsLayout
from tensorforce.agents import TRPOAgent
from tensorforce.execution import Runner
from rl.multi_agent_wrapper import MultiAgentWrapper
from tensorforce import Configuration
from rl.env_wrapper import EnvWrapper, OpenAIWrapper
import threading
from statistics import mean
EXIT = -1
NOOP = 0
SAVE = 1
flag = NOOP


class Learning:

    def __init__(
            self,
            dog_count=1,
            sheep_count=30,
            layout=AgentsLayout.DOGS_OUTSIDE_CIRCLE,
            max_episode_timesteps=3000,
            agent_type=TRPOAgent,
            batch_size=2048,
            repeat_actions=5,
            terminal_reward=0
    ):
        self.save_dir = os.path.dirname(__file__) + '/model/'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.network_spec = [
            dict(type='dense', size=128),
            dict(type='dense', size=64)
        ]
        params = EnvParams()
        params.DOG_COUNT = dog_count
        params.SHEEP_COUNT = sheep_count
        params.LAYOUT_FUNCTION = layout
        self.env = OpenAIWrapper(EnvWrapper(params), 'herding')
        self.configuration = Configuration(
                    batch_size=batch_size,
                    normalize_rewards=True
                )
        self.agent_type = agent_type
        self.agent = MultiAgentWrapper(
                self.agent_type,
                dict(
                    states_spec=self.env.states,
                    actions_spec=self.env.actions,
                    network_spec=self.network_spec,
                    config=self.configuration
                ),
                dog_count)

        self.repeat_actions = repeat_actions
        self.max_episode_timesteps = max_episode_timesteps
        self.runner = Runner(agent=self.agent, environment=self.env, repeat_actions=repeat_actions)
        self.instance_episodes = 0
        self.terminal_reward = terminal_reward
        sys.stdout.flush()

    def _log_data(self, r, info):
        with open(self.save_dir + '/out.log', 'a+') as f:
            message = '{ep} {ts} {rw} {info}\n'.format(ep=r.episode, ts=r.timestep, rw=r.episode_rewards[-1], info=info)
            f.write(message)
            print(message)
        sys.stdout.flush()

    def episode_finished(self, r):
        global flag, EXIT, SAVE, NOOP
        save_frequency = 50
        info = ''
        self.instance_episodes += 1

        if self.instance_episodes >= save_frequency and r.episode % save_frequency == 0:
            self.save_model()

        self._log_data(r, info)

        if flag == SAVE:
            self.save_model()
            return False
        if flag == EXIT:
            return False
        if len(r.episode_rewards) >= 50 and mean(r.episode_rewards[-50:]) > self.terminal_reward:
            self.save_model()
            return False

        return True

    def learn(self):
        self.runner.run(episode_finished=self.episode_finished, max_episode_timesteps=self.max_episode_timesteps)

    def stop_learning(self):
        self.agent.stop = True

    def load_model(self):
        if os.path.isfile(self.save_dir + '/checkpoint'):
            self.agent.load_model(self.save_dir)
            print('model loaded')
            sys.stdout.flush()

    def save_model(self):
        self.agent.save_model(self.save_dir)
        print('model saved')
        sys.stdout.flush()


class InputThread(threading.Thread):

    def run(self):
        global flag, EXIT, NOOP, SAVE
        while flag is not EXIT:
            text = input()
            if text is 'q':
                flag = EXIT
            if text is 's':
                flag = SAVE
                break


class LearningThread(threading.Thread):

    def __init__(self, params):
        super().__init__()
        self.learning = Learning(**params)

    def run(self):
        self.learning.load_model()
        self.learning.learn()
