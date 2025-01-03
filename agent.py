from rsoccer_gym.Entities import Robot
from utils.ssl.Navigation import Navigation
from utils.Point import Point
from utils.ssl.base_agent import BaseAgent
from itertools import permutations

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def decision(self):
        if len(self.targets) == 0:
            return

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

        return

    def post_decision(self):
        pass
