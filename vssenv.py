import numpy as np
import gymnasium as gym
import pygame
from rsoccer_gym.Utils.Utils import OrnsteinUhlenbeckAction
from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv
from typing import List
from utils.vss.Navigation import Navigation
from utils.Point import Point
from rsoccer_gym.Utils import KDTree
import random

TIME_STEP_DIFF = 0.16 / 0.025

class ExampleEnv(VSSBaseEnv):
    def __init__(self):
        super().__init__(
            field_type=0,
            n_robots_blue=3,
            n_robots_yellow=3,
            time_step=0.025,
            render_mode="human"
        )

        self.repeat_action = np.ceil(TIME_STEP_DIFF)
        self.reward_shaping_total = {}
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(
            low=-self.NORM_BOUNDS, high=self.NORM_BOUNDS, shape=(40,), dtype=np.float32
        )

        self.max_dist = self.field.rbt_radius * 10

        self.ou_actions = []
        for i in range(self.n_robots_blue + self.n_robots_yellow):
            self.ou_actions.append(
                OrnsteinUhlenbeckAction(self.action_space, dt=self.time_step)
            )

    def _frame_to_observations(self):
        observation = []

        observation.append(self.norm_pos(self.frame.ball.x))
        observation.append(self.norm_pos(self.frame.ball.y))
        observation.append(self.norm_v(self.frame.ball.v_x))
        observation.append(self.norm_v(self.frame.ball.v_y))

        for i in range(self.n_robots_blue):
            observation.append(self.norm_pos(self.frame.robots_blue[i].x))
            observation.append(self.norm_pos(self.frame.robots_blue[i].y))
            observation.append(np.sin(np.deg2rad(self.frame.robots_blue[i].theta)))
            observation.append(np.cos(np.deg2rad(self.frame.robots_blue[i].theta)))
            observation.append(self.norm_v(self.frame.robots_blue[i].v_x))
            observation.append(self.norm_v(self.frame.robots_blue[i].v_y))
            observation.append(self.norm_w(self.frame.robots_blue[i].v_theta))

        for i in range(self.n_robots_yellow):
            observation.append(self.norm_pos(self.frame.robots_yellow[i].x))
            observation.append(self.norm_pos(self.frame.robots_yellow[i].y))
            observation.append(np.sin(np.deg2rad(self.frame.robots_yellow[i].theta)))
            observation.append(np.cos(np.deg2rad(self.frame.robots_yellow[i].theta)))
            observation.append(self.norm_v(self.frame.robots_yellow[i].v_x))
            observation.append(self.norm_v(self.frame.robots_yellow[i].v_y))
            observation.append(self.norm_w(self.frame.robots_yellow[i].v_theta))

        return np.array(observation, dtype=np.float32)

    def _get_initial_positions_frame(self):
        """Returns the position of each robot and ball for the initial frame"""
        field_half_length = self.field.length / 2
        field_half_width = self.field.width / 2

        def x():
            return random.uniform(-field_half_length + 0.1, field_half_length - 0.1)

        def y():
            return random.uniform(-field_half_width + 0.1, field_half_width - 0.1)

        def theta():
            return random.uniform(0, 360)
        
        def v():
            return random.uniform(-self.max_v, self.max_v)

        pos_frame: Frame = Frame()

        pos_frame.ball = Ball(x=x(), y=y(), v_x=v(), v_y=v())

        min_dist = 0.1

        places = KDTree()
        places.insert((pos_frame.ball.x, pos_frame.ball.y))

        for i in range(self.n_robots_blue):
            pos = (x(), y())
            while places.get_nearest(pos)[1] < min_dist:
                pos = (x(), y())

            places.insert(pos)
            pos_frame.robots_blue[i] = Robot(x=pos[0], y=pos[1], theta=theta())
        

        for i in range(0, self.n_robots_yellow):
            pos = (x(), y())
            while places.get_nearest(pos)[1] < min_dist:
                pos = (x(), y())

            places.insert(pos)
            pos_frame.robots_yellow[i] = Robot(x=pos[0], y=pos[1], theta=theta())

        return pos_frame
    
    def actions_to_v_wheels(self, actions):
        left_wheel_speed = actions[0] * self.max_v
        right_wheel_speed = actions[1] * self.max_v

        left_wheel_speed, right_wheel_speed = np.clip(
            (left_wheel_speed, right_wheel_speed), -self.max_v, self.max_v
        )

        # Convert to rad/s
        left_wheel_speed /= self.field.rbt_wheel_radius
        right_wheel_speed /= self.field.rbt_wheel_radius

        return left_wheel_speed, right_wheel_speed   

    def actions_to_point(self, robot: Robot, actions):
        return Point(robot.x + actions[0]*self.max_dist, robot.y + actions[1]*self.max_dist) 

    def _get_commands(self, actions):
        commands = []
        
        robot = self.frame.robots_blue[0]

        target_point = self.actions_to_point(robot, actions)
        # follow ball
        ball = Point(self.frame.ball.x, self.frame.ball.y)
        target_point = Point(ball.x, ball.y)
        v_wheel0, v_wheel1 = Navigation.goToPoint(robot, target_point)
        
        commands.append(Robot(yellow=False, id=0, v_wheel0=v_wheel0, v_wheel1=v_wheel1))

        robot = self.frame.robots_blue[1]
        v_wheel0, v_wheel1 = Navigation.goToPoint(robot, target_point)
        commands.append(Robot(yellow=False, id=1, v_wheel0=v_wheel0, v_wheel1=v_wheel1))

        # Send random commands to the other robots
        # for i in range(1, self.n_robots_blue):
        #     actions = self.ou_actions[i].sample()
        #     v_wheel0, v_wheel1 = self.actions_to_v_wheels(actions)
        #     commands.append(
        #         Robot(yellow=False, id=i, v_wheel0=v_wheel0, v_wheel1=v_wheel1)
        #     )

        # for i in range(0, self.n_robots_yellow):
        #     actions = self.ou_actions[self.n_robots_blue + i].sample()
        #     v_wheel0, v_wheel1 = self.actions_to_v_wheels(actions)
        #     commands.append(
        #         Robot(yellow=True, id=i, v_wheel0=v_wheel0, v_wheel1=v_wheel1)
        #     )

        return commands

    def step(self, action):
        for _ in range(int(self.repeat_action)):
            self.steps += 1
            # Join agent action with environment actions
            commands: List[Robot] = self._get_commands(action)
            
            # Send command to simulator
            self.rsim.send_commands(commands)
            self.sent_commands = commands

            # Get Frame from simulator
            self.last_frame = self.frame
            self.frame = self.rsim.get_frame()

            # Calculate environment observation, reward and done condition
            observation = self._frame_to_observations()
            done = False


            if done:
                break

        return observation, 0, done, False, {}
    