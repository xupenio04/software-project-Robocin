from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.ssl.RRT import RRT
from rsoccer_gym.Render import SSLRenderField
import numpy as np
from utils.Point import Point

    

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.path =[]

    def predict_obstacle_positions(self, obstacles, prediction_time=0.1):
        """
        Prever a posição futura dos obstáculos com base em suas velocidades.
        """
        predicted_obstacles = []
        for obs_id, obs in obstacles.items():
            predicted_pos = Point(
                obs.x + obs.v_x * prediction_time,  # Usando v_x para velocidade no eixo x
                obs.y + obs.v_y * prediction_time   # Usando v_y para velocidade no eixo y
            )
            predicted_obstacles.append(predicted_pos)
        return predicted_obstacles

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
                max_iter=300,
                min_dist=0.175,
            )
            self.path = rrt.plan() or [] 



        if self.path and len(self.path) > 1:
            # Próximo ponto no caminho
            next_point = self.path[1]
            # Obter velocidades necessárias para atingir o próximo ponto
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, next_point)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)

            # Remover o ponto do caminho se já atingido
            if np.linalg.norm([current_pos.x - next_point.x, current_pos.y - next_point.y]) < 0.1:
                self.path.pop(0)
        else:
            # Caso o caminho esteja vazio ou o objetivo seja alcançado
            self.set_vel(Point(0, 0))
            self.set_angle_vel(0)


        return

    def post_decision(self):
        pass