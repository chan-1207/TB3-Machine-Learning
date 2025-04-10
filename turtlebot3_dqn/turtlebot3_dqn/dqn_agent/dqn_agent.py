#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import collections
import datetime
import json
import math
import numpy
import os
import random
import sys
import time

from keras.api.layers import Dense
from keras.api.models import load_model
from keras.api.models import Sequential
from keras.api.optimizers import Adam
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import tensorflow as tf

from turtlebot3_msgs.srv import Dqn


tf.config.set_visible_devices([], 'GPU')

LOGGING = False
current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
dqn_reward_log_dir = 'logs/gradient_tape/' + current_time + '/dqn_stage4_usual_180_ray_yaw_reward'

class DQNMetric(tf.keras.metrics.Metric):

    def __init__(self, name='dqn_metric'):
        super(DQNMetric, self).__init__(name=name)
        self.loss = self.add_weight(name='loss', initializer='zeros')
        self.episode_step = self.add_weight(name='step', initializer='zeros')

    def update_state(self, y_true, y_pred=0, sample_weight=None):
        self.loss.assign_add(y_true)
        self.episode_step.assign_add(1)

    def result(self):
        return self.loss / self.episode_step

    def reset_states(self):
        self.loss.assign(0)
        self.episode_step.assign(0)


class DQNAgent(Node):

    def __init__(self, stage):
        super().__init__('dqn_agent')

        self.stage = int(stage)
        self.train_mode = True
        # 수정: 환경에서 반환하는 state가 26개라면 state_size를 26으로 설정
        self.state_size = 26
        self.action_size = 5
        self.max_training_episodes = 10003

        # DQN 하이퍼파라미터 설정
        self.discount_factor = 0.99
        self.learning_rate = 0.0007
        self.epsilon = 1.0
        self.step_counter = 0
        self.epsilon_decay = 20000 * self.stage
        self.epsilon_min = 0.05
        self.batch_size = 128

        self.replay_memory = collections.deque(maxlen=500000)
        self.min_replay_memory_size = 5000

        # Q-Network 및 타겟 네트워크 생성, 초기 업데이트 수행
        self.model = self.create_qnetwork()
        self.target_model = self.create_qnetwork()
        self.update_target_model()
        self.update_target_after = 5000
        self.target_update_after_counter = 0

        self.load_model = False
        self.load_episode = 0
        self.model_dir_path = os.path.dirname(os.path.realpath(__file__))
        self.model_dir_path = self.model_dir_path.replace('turtlebot3_dqn/dqn_agent', 'model')
        self.model_path = os.path.join(
            self.model_dir_path, 'stage' + str(self.stage) + '_episode' + str(self.load_episode) + '.h5')

        if self.load_model:
            self.model.set_weights(load_model(self.model_path).get_weights())
            with open(os.path.join(
                self.model_dir_path,
                'stage' + str(self.stage) + '_episode' + str(self.load_episode) + '.json'
            )) as outfile:
                param = json.load(outfile)
                self.epsilon = param.get('epsilon')

        if LOGGING:  # Tensorboard Log
            self.dqn_reward_writer = tf.summary.create_file_writer(dqn_reward_log_dir)
            self.dqn_reward_metric = DQNMetric()

        self.rl_agent_interface_client = self.create_client(Dqn, 'rl_agent_interface')
        self.make_environment_client = self.create_client(Empty, 'make_environment')
        self.reset_environment_client = self.create_client(Dqn, 'reset_environment')

        self.process()

    def process(self):
        self.env_make()
        time.sleep(1.0)

        episode_num = 0

        for episode in range(self.load_episode + 1, self.max_training_episodes):
            episode_num += 1
            local_step = 0
            score = 0

            state = self.reset_environment()  # 환경 리셋 후 초기 상태 획득
            time.sleep(1.0)

            while True:
                local_step += 1
                action = int(self.get_action(state))  # epsilon-greedy 정책에 따른 행동 선택

                next_state, reward, done = self.step(action)  # 선택한 행동 수행 및 결과 획득
                score += reward

                if self.train_mode:
                    self.append_sample((state, action, reward, next_state, done))  # 경험 저장
                    self.train_model(done)  # 모델 학습

                state = next_state
                if done:
                    if LOGGING:
                        self.dqn_reward_metric.update_state(score)
                        with self.dqn_reward_writer.as_default():
                            tf.summary.scalar('dqn_reward', self.dqn_reward_metric.result(), step=episode_num)
                        self.dqn_reward_metric.reset_states()

                    print(
                        "Episode:", episode,
                        "score:", score,
                        "memory length:", len(self.replay_memory),
                        "epsilon:", self.epsilon)

                    param_keys = ['epsilon']
                    param_values = [self.epsilon]
                    param_dictionary = dict(zip(param_keys, param_values))
                    break

                time.sleep(0.01)

            # Update result and save model every 100 episodes
            if self.train_mode:
                if episode % 100 == 0:
                    self.model_path = os.path.join(
                        self.model_dir_path,
                        'stage' + str(self.stage) + '_episode' + str(episode) + '.h5')
                    self.model.save(self.model_path)
                    with open(os.path.join(
                            self.model_dir_path,
                            'stage' + str(self.stage) + '_episode' + str(episode) + '.json'), 'w') as outfile:
                        json.dump(param_dictionary, outfile)

    def env_make(self):  # env_make: ROS2 서비스 클라이언트를 통해 환경 생성 요청을 보낸다.
        while not self.make_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Environment make client failed to connect to the server, try again ...')

        self.make_environment_client.call_async(Empty.Request())

    def reset_environment(self):  # reset_environment: ROS2 서비스를 통해 환경을 리셋하고, 초기 상태(state)를 반환한다.
        while not self.reset_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Reset environment client failed to connect to the server, try again ...')

        future = self.reset_environment_client.call_async(Dqn.Request())

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            state = future.result().state
            state = numpy.reshape(numpy.asarray(state), [1, self.state_size])
        else:
            self.get_logger().error(
                'Exception while calling service: {0}'.format(future.exception()))

        return state

    def get_action(self, state):  # epsilon-greedy, 지수감쇠
        if self.train_mode:
            self.step_counter += 1
            self.epsilon = self.epsilon_min + (1.0 - self.epsilon_min) * math.exp(
                -1.0 * self.step_counter / self.epsilon_decay)
            lucky = random.random()
            if lucky > (1 - self.epsilon): # 앱실론 확률에 의해 (랜덤 행동 / 학습된 최적 행동) 선택
                return random.randint(0, self.action_size - 1)
            else:
                return numpy.argmax(self.model.predict(state))
        else:
            return numpy.argmax(self.model.predict(state))

    def step(self, action):  # step: 현재 상태에서 선택된 행동을 환경에 전달, (다음상태, 보상, 종료여부)를 반환받음
        req = Dqn.Request()
        req.action = action

        while not self.rl_agent_interface_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('rl_agent interface service not available, waiting again...')

        future = self.rl_agent_interface_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            next_state = future.result().state
            next_state = numpy.reshape(numpy.asarray(next_state), [1, self.state_size])
            reward = future.result().reward
            done = future.result().done
        else:
            self.get_logger().error(
                'Exception while calling service: {0}'.format(future.exception()))

        return next_state, reward, done

# create_qnetwork: Keras Sequential 모델을 사용하여 Q-Network를 생성하고 컴파일
    def create_qnetwork(self):
        model = Sequential()
        model.add(Dense(512, input_shape=(self.state_size,), activation='relu'))
        model.add(Dense(256, activation='relu'))
        model.add(Dense(128, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse', optimizer=Adam(learning_rate=self.learning_rate))
        model.summary()

        return model

    def update_target_model(self):  # 현재 Q-Network의 가중치를 타겟 네트워크에 복사하고 업데이트 카운터를 초기화
        self.target_model.set_weights(self.model.get_weights())
        self.target_update_after_counter = 0
        print("*Target model updated*")

    def append_sample(self, transition):  # append_sample: (상태, 행동, 보상, 다음 상태, 종료여부)로 구성된 경험을 리플레이 메모리에 추가
        self.replay_memory.append(transition)

    def train_model(self, terminal):  # 충분한 경험이 쌓이면 미니배치 샘플링 후 Q-Network를 학습시키고, 에피소드 종료 시 타겟 네트워크 업데이트를 수행한다.
        if len(self.replay_memory) < self.min_replay_memory_size:
            return
        data_in_mini_batch = random.sample(self.replay_memory, self.batch_size)

        current_states = numpy.array([transition[0] for transition in data_in_mini_batch])
        current_states = current_states.squeeze()
        current_qvalues_list = self.model.predict(current_states)

        next_states = numpy.array([transition[3] for transition in data_in_mini_batch])
        next_states = next_states.squeeze()
        next_qvalues_list = self.target_model.predict(next_states)

        x_train = []
        y_train = []

        for index, (current_state, action, reward, next_state, done) in enumerate(data_in_mini_batch):
            if not done:
                future_reward = numpy.max(next_qvalues_list[index])
                desired_q = reward + self.discount_factor * future_reward
            else:
                desired_q = reward

            current_q_values = current_qvalues_list[index]
            current_q_values[action] = desired_q

            x_train.append(current_state)
            y_train.append(current_q_values)

        x_train = numpy.array(x_train)
        y_train = numpy.array(y_train)
        x_train = numpy.reshape(x_train, [len(data_in_mini_batch), self.state_size])
        y_train = numpy.reshape(y_train, [len(data_in_mini_batch), self.action_size])

        self.model.fit(
            tf.convert_to_tensor(x_train, tf.float32),
            tf.convert_to_tensor(y_train, tf.float32),
            batch_size=self.batch_size, verbose=0
        )
        self.target_update_after_counter += 1

        if self.target_update_after_counter > self.update_target_after and terminal:
            self.update_target_model()


def main(args=None):
    if args is None:
        args = sys.argv
    stage = args[1] if len(args) > 1 else '1'
    rclpy.init(args=args)

    dqn_agent = DQNAgent(stage)
    rclpy.spin(dqn_agent)

    dqn_agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
