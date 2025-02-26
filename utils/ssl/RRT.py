import numpy as np
from utils.Point import Point
import random

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
        self.difficulty = difficulty
        self.speed_factor = 1.0  
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
        self.step_size = self.adjust_step_size(from_point)
        theta = np.arctan2(to_point.y - from_point.y, to_point.x - from_point.x)
        return Point(from_point.x + self.step_size * np.cos(theta), from_point.y + self.step_size * np.sin(theta))

    def adjust_speed_factor(self, point):
        safe_distance = 0.5  
        min_speed = 0.3      
        max_speed = 1.0     

        closest_distance = min(
            self.distance(point, obs) for obs in self.obstacles
        )

        if closest_distance >= safe_distance:
            self.speed_factor = max_speed
        else:
            self.speed_factor = min_speed + (max_speed - min_speed) * (closest_distance / safe_distance)

    def is_collision_free_with_subdivision(self, p1, p2, subdivisions):
        for i in range(1, subdivisions + 1):
            t = i / subdivisions
            intermediate_point = Point(
                p1.x + t * (p2.x - p1.x),
                p1.y + t * (p2.y - p1.y)
            )
            self.adjust_speed_factor(intermediate_point) 
            if not self.is_collision_free(p1, intermediate_point):
                return False
        return True

    def adjust_step_size(self, point):
        safe_distance = 0.5
        min_step = 0.05
        max_step = 0.2
        closest_distance = min(self.distance(point, obs) for obs in self.obstacles)
        if closest_distance > safe_distance:
            return max_step
        return min_step + (max_step - min_step) * (closest_distance / safe_distance)


    def random_point_weighted_by_obstacle_density(self):
        num_attempts = 150  
        for _ in range(num_attempts):
            rand_x = random.uniform(self.x_bounds[0], self.x_bounds[1])
            rand_y = random.uniform(self.y_bounds[0], self.y_bounds[1])

            distance_to_nearest_obstacle = min(self.distance(Point(rand_x, rand_y), obs) for obs in self.obstacles)

            if distance_to_nearest_obstacle > self.min_dist:
                return Point(rand_x, rand_y)

        return Point(rand_x, rand_y)

    def plan(self):
        for _ in range(self.max_iter):
            rand_point = self.random_point_weighted_by_obstacle_density()

            if self.distance(rand_point, self.goal) < self.step_size and self.is_collision_free(self.nearest(rand_point), self.goal):
                self.tree[self.goal] = self.nearest(rand_point)
                raw_path = self.reconstruct_path()
                return self.smooth_path(raw_path)

            nearest_point = self.nearest(rand_point)
            new_point = self.steer(nearest_point, rand_point)

            if self.is_collision_free(nearest_point, new_point):
                self.tree[new_point] = nearest_point

                if self.distance(new_point, self.goal) < self.step_size:
                    self.tree[self.goal] = new_point
                    raw_path = self.reconstruct_path()
                    return self.smooth_path(raw_path)

        return None


    def reconstruct_path(self):
        path = []
        current = self.goal
        while current is not None:
            path.append(current)
            current = self.tree.get(current)
        path.reverse()
        return path

    def smooth_path(self, path):
        if self.difficulty == 1:  
            return self.smooth_path_with_interpolation(path)
        else:
            return self.smooth_path_based_on_density(path)

    def smooth_path_with_interpolation(self, path, interpolation_step=0.01):
        smoothed_path = [path[0]]
        for i in range(1, len(path)):
            if self.is_collision_free(smoothed_path[-1], path[i]):
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


    def smooth_path_with_subdivisions_simple(self, path, subdivision_factor=5, avoidance_radius=0.2):
        if len(path) < 5:
            return path

        smoothed_path = [path[0]]
        for i in range(1, len(path)):
            current_point = path[i]

            density = self.obstacle_density(current_point, avoidance_radius)

            if density > 0:
                alternative_point = self.find_alternative_point(current_point, radius=avoidance_radius)
                if alternative_point:
                    current_point = alternative_point

            if self.is_collision_free_with_subdivision(smoothed_path[-1], current_point, subdivision_factor):
                smoothed_path.append(current_point)
            else:
                smoothed_path.append(path[i - 1])

        smoothed_path.append(path[-1])
        return smoothed_path
    
    def obstacle_density(self, point, radius=0.5):
        count = 0
        for obs in self.obstacles:
            if self.distance(point, obs) < radius:
                count += 1
        return count
    
    def find_alternative_point(self, point, radius=0.2, step_size=0.05, max_attempts=5):
        best_point = None
        best_distance = float('inf')

        attempts = 0
        while attempts < max_attempts:
            angle = np.random.uniform(0, 2 * np.pi)
            new_x = point.x + step_size * np.cos(angle)
            new_y = point.y + step_size * np.sin(angle)

            if self.x_bounds[0] <= new_x <= self.x_bounds[1] and self.y_bounds[0] <= new_y <= self.y_bounds[1]:
                alternative_point = Point(new_x, new_y)

                if self.is_collision_free_with_subdivision(point, alternative_point, 1):
                    distance_to_goal = self.distance(alternative_point, point)
                    if distance_to_goal < best_distance:
                        best_distance = distance_to_goal
                        best_point = alternative_point

            attempts += 1

        return best_point
     
    def smooth_path_based_on_density(self, path, subdivision_factor=5, avoidance_radius=0.2):
        if len(path) < 5:
            return path

        high_density = self.check_path_density(path, avoidance_radius)

        if high_density:
            return self.smooth_path_with_subdivisions_simple(path, subdivision_factor, avoidance_radius)
        else:
            return self.smooth_path_with_subdivisions(path, subdivision_factor)

    def check_path_density(self, path, avoidance_radius=0.2):
        high_density_count = 0
        for point in path:
            if self.obstacle_density(point, avoidance_radius) > 0:
                high_density_count += 1

        density_threshold = 0.5
        if high_density_count / len(path) > density_threshold:
            return True
        return False

    def smooth_path_with_subdivisions(self, path, subdivision_factor=5): 
        if len(path) < 5:
            return path

        smoothed_path = [path[0]]
        for i in range(2, len(path)):
            if self.is_collision_free_with_subdivision(smoothed_path[-1], path[i], subdivision_factor):
                continue
            smoothed_path.append(path[i - 1])
        smoothed_path.append(path[-1])
        return smoothed_path

   