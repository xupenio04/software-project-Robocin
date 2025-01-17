import numpy as np
from utils.Point import Point
import random
from scipy.spatial import KDTree
import argparse
import textwrap
from utils.CLI import Difficulty

class RRT:
    def __init__(self, start, goal, obstacles, x_bounds, y_bounds, step_size=0.1, max_iter=1000, min_dist=0.18, difficulty=1):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.min_dist = min_dist
        self.difficulty = difficulty  # Novo parâmetro para controlar a dificuldade
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
    
    def steer(self, from_point, to_point):
        dist = self.distance(from_point, to_point)
        if dist < self.step_size:
            return to_point
        theta = np.arctan2(to_point.y - from_point.y, to_point.x - from_point.x)
        return Point(from_point.x + self.step_size * np.cos(theta), from_point.y + self.step_size * np.sin(theta))
    
    def plan(self):
        for _ in range(self.max_iter):
            rand_point = Point(
                random.uniform(self.x_bounds[0], self.x_bounds[1]),
                random.uniform(self.y_bounds[0], self.y_bounds[1])
            )

            nearest_point = self.nearest(rand_point)
            new_point = self.steer(nearest_point, rand_point)

            if self.is_collision_free(nearest_point, new_point):
                self.tree[new_point] = nearest_point

                if self.distance(new_point, self.goal) < self.step_size:
                    self.tree[self.goal] = new_point
                    raw_path = self.reconstruct_path()
                    return self.smooth_path(raw_path)  # Suavizar o caminho encontrado
        return None
    
    def smooth_path(self, path):
           
        if self.difficulty == 1:  
            return self.smooth_path_with_interpolation(path)
        else:  
            return self.smooth_path_with_subdivisions(path)
        
    def smooth_path_with_interpolation(self, path, interpolation_step=0.01):
        smoothed_path = [path[0]]
        for i in range(1, len(path)):
            if self.is_collision_free(smoothed_path[-1], path[i]):
                # Interpolar pontos entre smoothed_path[-1] e path[i]
                start = smoothed_path[-1]
                end = path[i]
                distance = self.distance(start, end)
                steps = int(distance / interpolation_step)
                for j in range(1, steps):
                    interp_point = Point(
                        start.x + (end.x - start.x) * j / steps,
                        start.y + (end.y - start.y) * j / steps
                    )
                    smoothed_path.append(interp_point)
        smoothed_path.append(path[-1])
        return smoothed_path


        
    def smooth_path_with_subdivisions(self, path, subdivision_factor=5):
      
        if len(path) < 3:  # Se o caminho for muito curto, não precisa suavizar.
            return path

        smoothed_path = [path[0]]

        for i in range(2, len(path)):
            # Verificar o caminho direto entre o último ponto suavizado e o ponto atual.
            if self.is_collision_free_with_subdivision(smoothed_path[-1], path[i], subdivision_factor):
                continue  # Pula o ponto intermediário se o caminho direto for possível.
            smoothed_path.append(path[i - 1])  # Adiciona o ponto intermediário.

        smoothed_path.append(path[-1])  # Adiciona o ponto final.
        return smoothed_path

    def is_collision_free_with_subdivision(self, p1, p2, subdivisions):
    
        for i in range(1, subdivisions + 1):
            # Gera pontos intermediários ao longo do caminho.
            t = i / subdivisions
            intermediate_point = Point(
                p1.x + t * (p2.x - p1.x),
                p1.y + t * (p2.y - p1.y)
            )
            if not self.is_collision_free(p1, intermediate_point):
                return False
        return True

    def reconstruct_path(self):
        path = []
        current = self.goal
        while current is not None:
            path.append(current)
            current = self.tree.get(current)
        path.reverse()
        return self.smooth_path(path)
    

