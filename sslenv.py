import numpy as np
from gymnasium.spaces import Box
from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.ssl.ssl_gym_base import SSLBaseEnv
from rsoccer_gym.Utils import KDTree
from utils.ssl.Navigation import Navigation
from utils.Point import Point
from utils.ssl.small_field import SSLHRenderField
from agent import ExampleAgent
from random_agent import RandomAgent
import random
import pygame


class SSLExampleEnv(SSLBaseEnv):
    def __init__(self, render_mode="human"):
        field = 2   # 1: SSL Div B    2: SSL Hardware challenge
        super().__init__(
            field_type=field, 
            n_robots_blue=11,
            n_robots_yellow=11, 
            time_step=0.025, 
            render_mode=render_mode)
        
        n_obs = 4 # Ball x,y and Robot x, y
        self.action_space = Box(low=-1, high=1, shape=(2, ))
        self.observation_space = Box(low=-self.field.length/2,\
            high=self.field.length/2,shape=(n_obs, ))
        
        self.target = Point(0,0)
        self.min_dist = 0.18
        self.all_points = []
        self.robot_path = []
        self.agent = ExampleAgent(0, False)     ## Um único agente por enquanto

        self.blue_agents = {i: RandomAgent(i, False) for i in range(1, 11)}
        self.yellow_agents = {i: RandomAgent(i, True) for i in range(0, 11)}

        if field == 2:
            self.field_renderer = SSLHRenderField()
            self.window_size = self.field_renderer.window_size
        
    def _frame_to_observations(self):
        ball, robot = self.frame.ball, self.frame.robots_blue[0]
        return np.array([ball.x, ball.y, robot.x, robot.y])

    def _get_commands(self, actions):
        robot = self.frame.robots_blue[0]
        robot_pos = Point(x=robot.x,
                          y=robot.y)

        current_target = self.target
        self.all_points.append(current_target)
        self.robot_path.append(robot_pos)

        # Keep only the last M target points
        max_target_points = 300
        if len(self.all_points) > max_target_points:
            self.all_points.pop(0)

        # Visible path drawing control
        max_path_length = 300
        if len(self.robot_path) > max_path_length:
            self.robot_path.pop(0)

        if robot_pos.dist_to(self.target) < self.min_dist:
            self.target = Point(x=self.x(), y=self.y())
            
        opponents = {id: robot for id, robot in self.frame.robots_yellow.items()}
        for i in range(0, self.n_robots_blue):
            if (i != robot.id):
                opponents[i + self.n_robots_yellow] = self.frame.robots_blue[i]

        action = self.agent.step(robot, opponents, self.target)

        others_actions = []
        for i in range(1, self.n_robots_blue):
            if random.uniform(0.0, 1.0) < 0.003:
                random_target = Point(x=self.x(), y=self.y())  
            else:
                random_target = None

            others_actions.append(self.blue_agents[i].step(self.frame.robots_blue[i], opponents, random_target))

        for i in range(0, self.n_robots_yellow):
            if random.uniform(0.0, 1.0) < 0.003:
                random_target = Point(x=self.x(), y=self.y())
            else:
                random_target = None

            others_actions.append(self.yellow_agents[i].step(self.frame.robots_yellow[i], opponents, random_target))

        return [action]  + others_actions

    def _calculate_reward_and_done(self):
        if self.frame.ball.x > self.field.length / 2 \
            and abs(self.frame.ball.y) < self.field.goal_width / 2:
            reward, done = 1, True
        else:
            reward, done = 0, False
        return reward, done
    
    def x(self):
        return random.uniform(-self.field.length/2 + self.min_dist, self.field.length/2 - self.min_dist)

    def y(self):
        return random.uniform(-self.field.width/2 + self.min_dist, self.field.width/2 - self.min_dist)
    
    def _get_initial_positions_frame(self):

        def theta():
            return random.uniform(0, 360)
    
        pos_frame: Frame = Frame()

        pos_frame.ball = Ball(x=self.x(), y=self.y())

        pos_frame.robots_blue[0] = Robot(x=self.x(), y=self.y(), theta=theta())

        self.target = Point(x=self.x(), y=self.y())

        places = KDTree()
        places.insert((pos_frame.ball.x, pos_frame.ball.y))

        for i in range(self.n_robots_blue):
            pos = (self.x(), self.y())
            while places.get_nearest(pos)[1] < self.min_dist:
                pos = (self.x(), self.y())

            places.insert(pos)
            pos_frame.robots_blue[i] = Robot(x=pos[0], y=pos[1], theta=theta())
        

        for i in range(0, self.n_robots_yellow):
            pos = (self.x(), self.y())
            while places.get_nearest(pos)[1] < self.min_dist:
                pos = (self.x(), self.y())

            places.insert(pos)
            pos_frame.robots_yellow[i] = Robot(x=pos[0], y=pos[1], theta=theta())

        return pos_frame
    

    def _render(self):
        def pos_transform(pos_x, pos_y):
            return (
                int(pos_x * self.field_renderer.scale + self.field_renderer.center_x),
                int(pos_y * self.field_renderer.scale + self.field_renderer.center_y),
            )

        super()._render()
        
        self.draw_target(
            self.window_surface,
            pos_transform,
            self.target,
            (255, 0, 255),
        )

        if len(self.all_points) > 1:
            my_path = [pos_transform(*p) for p in self.all_points[:-1]]
            for point in my_path:
                pygame.draw.circle(self.window_surface, (255, 0, 0), point, 3)
        
        if len(self.robot_path) > 1:
            my_path = [pos_transform(*p) for p in self.robot_path]
            pygame.draw.lines(self.window_surface, (255, 0, 0), False, my_path, 1)

    def draw_target(self, screen, transformer, point, color):
        x, y = transformer(point.x, point.y)
        size = 0.09 * self.field_renderer.scale
        pygame.draw.circle(screen, color, (x, y), size, 2)