from scipy.optimize import linear_sum_assignment
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.ssl.RRT import RRT
from rsoccer_gym.Render import SSLRenderField
import numpy as np
from utils.Point import Point
import argparse
from utils.CLI import cli, Difficulty

args = cli()

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, difficulty=Difficulty(args.difficulty), allocation_method="hungarian"):
        super().__init__(id, yellow)
        self.path = []
        self.difficulty = difficulty
        self.allocation_method = allocation_method  

    def predict_obstacle_positions(self, obstacles, prediction_time=0.1):
      
        predicted_obstacles = []
        for obs_id, obs in obstacles.items():
            predicted_pos = Point(
                obs.x + obs.v_x * prediction_time,  
                obs.y + obs.v_y * prediction_time   
            )
            predicted_obstacles.append(predicted_pos)
        return predicted_obstacles



    def greedy_allocation(self, robots, targets):
        if not robots or not targets:
            return {}

        allocation = {}
        remaining_targets = set(targets)  
        robot_list = list(robots.items())  

        while robot_list and remaining_targets:
            best_match = None
            min_distance = float('inf')

            for rob_id, rob_pos in robot_list:
                for target in remaining_targets:
                    distance = np.linalg.norm([rob_pos.x - target.x, rob_pos.y - target.y])
                    if distance < min_distance:
                        min_distance = distance
                        best_match = (rob_id, target)

            if best_match:
                rob_id, target = best_match
                allocation[rob_id] = target
                remaining_targets.remove(target)
                robot_list.remove((rob_id, robots[rob_id]))

        return allocation


    
    def allocate_tasks(self, robots, targets, obstacles):
        
        return self.greedy_allocation(robots, targets)

    def decision(self):
        if len(self.targets) == 0:
            return

        robots = {rob_id: rob for rob_id, rob in self.teammates.items()}
        targets = self.targets  # Lista de destinos
        obstacles = {
            rob_id: rob for rob_id, rob in self.opponents.items()
        } | {
            rob_id: rob for rob_id, rob in self.teammates.items() if rob_id != self.id
        }

        allocation = self.allocate_tasks(robots, targets, obstacles)

        if self.id not in allocation:
            self.set_vel(Point(0, 0))
            self.set_angle_vel(0)
            return

        goal = allocation[self.id]
        if not self.path or len(self.path) == 0 or self.path[-1] != goal:
            x_bounds = (-SSLRenderField.length / 2, SSLRenderField.length / 2)
            y_bounds = (-SSLRenderField.width / 2, SSLRenderField.width / 2)

            rrt = RRT(
                start=self.pos,
                goal=goal,
                obstacles=self.predict_obstacle_positions(obstacles),
                x_bounds=x_bounds,
                y_bounds=y_bounds,
                step_size=0.10,
                max_iter=300,
                min_dist=0.175,
                difficulty=self.difficulty.value
            )
            raw_path = rrt.plan()
            if raw_path:
                self.path = raw_path 

        if self.path and len(self.path) > 1:
            next_point = self.path[1]
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, next_point)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)

            if np.linalg.norm([self.pos.x - next_point.x, self.pos.y - next_point.y]) < 0.1:
                self.path.pop(0)
        else:
            self.set_vel(Point(0, 0))
            self.set_angle_vel(0)

    def post_decision(self):
        pass
