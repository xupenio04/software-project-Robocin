from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.ssl.RRT import RRT
from rsoccer_gym.Render import SSLRenderField
import numpy as np
from utils.Point import Point

    

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def decision(self):
        if len(self.targets) == 0:
            return

        current_pos = self.pos

        goal = self.targets[0]

        x_bounds = (-SSLRenderField.length / 2, SSLRenderField.length / 2)
        y_bounds = (-SSLRenderField.width / 2, SSLRenderField.width / 2)

        obstacles = [
            Point(rob.x, rob.y)
            for rob in self.opponents.values()
        ] + [
            Point(rob.x, rob.y)
            for rob_id, rob in self.teammates.items()
            if rob_id != self.id
        ]

        if self.path is None or len(self.path) == 0 or self.path[-1] != goal:
            rrt = RRT(
                start=current_pos,
                goal=goal,
                obstacles=obstacles,
                x_bounds=x_bounds,
                y_bounds=y_bounds,
                step_size=0.1,
                max_iter=1000,
                min_dist=0.18,
            )
            self.path = rrt.plan() or [] 



        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

        return

    def post_decision(self):
        pass