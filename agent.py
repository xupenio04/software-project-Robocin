from scipy.optimize import linear_sum_assignment
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.ssl.RRT import RRT
from rsoccer_gym.Render import SSLRenderField
import numpy as np
from utils.Point import Point
import argparse
from utils.CLI import cli, Difficulty

args= cli()

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, difficulty= Difficulty(args.difficulty) ):
        super().__init__(id, yellow)
        self.path = []
        self.difficulty = difficulty

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

    def allocate_tasks(self, robots, targets, obstacles):
       
        num_robots = len(robots)
        num_targets = len(targets)

        # Garantir que o número de robôs e destinos seja compatível
        if num_robots == 0 or num_targets == 0:
            return {}

        # Matriz de custos baseada na distância entre robôs e destinos
        cost_matrix = np.zeros((num_robots, num_targets))
        for i, (rob_id, rob_pos) in enumerate(robots.items()):
            for j, target in enumerate(targets):
                dist = np.linalg.norm([rob_pos.x - target.x, rob_pos.y - target.y])
                cost_matrix[i, j] = dist  # Penalidades podem ser adicionadas aqui

        # Resolver problema de atribuição (Algoritmo Húngaro)
        row_idx, col_idx = linear_sum_assignment(cost_matrix)

        # Criar mapeamento de robôs para destinos
        allocation = {list(robots.keys())[i]: targets[j] for i, j in zip(row_idx, col_idx)}

        return allocation

    def decision(self):
        if len(self.targets) == 0:
            return

        # Obter posições atuais dos robôs e destinos
        robots = {rob_id: rob for rob_id, rob in self.teammates.items()}
        targets = self.targets  # Lista de destinos
        obstacles = {
            rob_id: rob for rob_id, rob in self.opponents.items()
        } | {
            rob_id: rob for rob_id, rob in self.teammates.items() if rob_id != self.id
        }

        # Alocar destinos para robôs
        allocation = self.allocate_tasks(robots, targets, obstacles)

        # Verificar se este robô tem um destino atribuído
        if self.id not in allocation:
            self.set_vel(Point(0, 0))
            self.set_angle_vel(0)
            return

        # Planejar caminho para o destino atribuído
        goal = allocation[self.id]
        if not self.path or len(self.path) == 0 or self.path[-1] != goal:
            x_bounds = (-SSLRenderField.length / 2, SSLRenderField.length / 2)
            y_bounds = (-SSLRenderField.width / 2, SSLRenderField.width / 2)

            # Utilizar o novo RRT com suavização
            rrt = RRT(
                start=self.pos,
                goal=goal,
                obstacles=self.predict_obstacle_positions(obstacles),
                x_bounds=x_bounds,
                y_bounds=y_bounds,
                step_size=0.10,
                max_iter=250,
                min_dist=0.175,
                difficulty=self.difficulty.value
            )
            raw_path = rrt.plan()
            if raw_path:
                self.path = raw_path  # Caminho suavizado

        # Navegar no caminho planejado
        if self.path and len(self.path) > 1:
            next_point = self.path[1]
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, next_point)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)

            # Remover ponto alcançado do caminho
            if np.linalg.norm([self.pos.x - next_point.x, self.pos.y - next_point.y]) < 0.1:
                self.path.pop(0)
        else:
            # Parar se o objetivo for alcançado
            self.set_vel(Point(0, 0))
            self.set_angle_vel(0)

        return


    def post_decision(self):
        pass
