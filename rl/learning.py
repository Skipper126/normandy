import sys
import os
from env.herding import Herding, EnvParams, RotationMode, AgentsLayout
from tensorforce.agents import TRPOAgent
from tensorforce.execution import Runner
from rl.multi_agent_wrapper import MultiAgentWrapper
from tensorforce import Configuration
from rl.env_wrapper import EnvWrapper, OpenAIWrapper
from statistics import mean
import threading

EXIT = -1
NOOP = 0
SAVE = 1
flag = NOOP


class Learning:

    def __init__(
            self,
            network_spec=None,
            dog_count=1,
            sheep_count=20,
            layout=AgentsLayout.DOGS_OUTSIDE_CIRCLE,
            max_episode_timesteps=3000,
            agent_type=TRPOAgent,
            batch_size=2048,
            repeat_actions=5,
            save_dir=None,
    ):
        if save_dir is None:
            raise Exception("save dir required!")
        else:
            self.save_dir = './model/' + save_dir + '/'

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        if network_spec is None:
            self.network_spec = [
                dict(type='dense', size=128),
                dict(type='dense', size=64)
            ]
        else:
            self.network_spec = network_spec

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
        self.level_changed = False
        sys.stdout.flush()

    def _log_data(self, r, avg_reward, info):
        with open(self.save_dir + '/log.txt', 'a+') as f:
            message = '{ep} {ts} {ar} {info}\n'.format(ep=r.episode, ts=r.timestep, ar=avg_reward, info=info)
            f.write(message)
            print(message)
        sys.stdout.flush()

    def change_env(self, dog_count, sheep_count, layout):
        params = EnvParams()
        params.DOG_COUNT = dog_count
        params.SHEEP_COUNT = sheep_count
        params.LAYOUT_FUNCTION = layout
        self.save_model()
        self.env = OpenAIWrapper(EnvWrapper(params), 'herding')
        self.agent = MultiAgentWrapper(
            self.agent_type,
            dict(
                states_spec=self.env.states,
                actions_spec=self.env.actions,
                network_spec=self.network_spec,
                config=self.configuration
            ),
            dog_count
        )
        self.load_model()

    def episode_finished(self, r):
        global flag, EXIT, SAVE, NOOP
        save_frequency = 50
        avg_reward = mean(r.episode_rewards[-50:])
        info = ''

        if self.instance_episodes > save_frequency / 2 and r.episode % save_frequency == 0:
            self.save_model()
            self.instance_episodes += 1

        if self.level_changed is False and avg_reward > 0:
            self.change_env(dog_count=3, sheep_count=30, layout=AgentsLayout.DOGS_OUTSIDE_CIRCLE)
            self.level_changed = True
            info = 'level_changed'

        self._log_data(r, avg_reward, info)

        if flag == SAVE:
            self.save_model()
            return False
        if flag == EXIT:
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


def main():

    it = InputThread()
    configurations = [
        LearningThread(dict(
            save_dir='firstConf'
        )),
    ]
    it.start()
    for conf in configurations:
        conf.start()

    it.join()
    for conf in configurations:
        conf.join()


if __name__ == '__main__':
    main()
