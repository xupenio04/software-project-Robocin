import numpy as np
from utils.Point import Point
import random

class RRT:
    def __init__(self, start, goal, obstacles, x_bounds, y_bounds, step_size=0.1, max_iter=1000, min_dist=0.18):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.min_dist = min_dist
        self.tree = {start: None}  

    def distance(self, p1, p2):
        return np.linalg.norm([p1.x - p2.x, p1.y - p2.y])

    def is_collision_free(self, p1, p2):
        for obs in self.obstacles:
            if self.distance_to_segment(obs, p1, p2) < self.min_dist:
                return False
        return True

    def distance_to_segment(self, obs, p1, p2):
        px, py = obs.x, obs.y
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        line_mag = self.distance(p1, p2)
        if line_mag < 1e-6:
            return self.distance(obs, p1)
        t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_mag**2))
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        return np.linalg.norm([px - proj_x, py - proj_y])
    
    def nearest(self, point):
        return min(self.tree.keys(), key=lambda p: self.distance(p, point))