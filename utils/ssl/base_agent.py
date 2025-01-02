from rsoccer_gym.Entities import Robot
from utils.Point import Point

class BaseAgent:
    """Abstract Agent."""

    def __init__(self, id=0, yellow=False):
        self.id = id
        self.robot = Robot()
        self.pos = Point(0, 0)
        self.vel = Point(0, 0)
        self.body_angle = float(0)
        self.target = None
        self.yellow = yellow
        self.opponents = dict()

        self.next_vel = Point(0, 0)
        self.angle_vel = float(0)

    def step(self, self_robot : Robot, opponents: dict[int, Robot], target: Point = None) -> Robot:
        inverter = -1 

        self.reset()
        self.pos = Point(self_robot.x, self_robot.y * inverter)
        self.vel = Point(self_robot.v_x, self_robot.v_y * inverter)
        self.body_angle = self_robot.theta
        if target is not None:
            self.target = Point(target.x, target.y * inverter)
        self.robot = Robot(id=self_robot.id, yellow=self.yellow,
                           x=self.pos.x, y=self.pos.y,
                           v_x=self.vel.x, v_y=self.vel.y, theta=self.body_angle)
        
        for id, robot in opponents.items():
            self.opponents[id] = Robot(id=robot.id, yellow=robot.yellow,
                                       x=robot.x, y=robot.y * inverter,
                                       v_x=robot.v_x, v_y=robot.v_y * inverter,
                                       theta=robot.theta)

        self.decision()
        self.post_decision()

        return Robot( id=self.id, yellow=self.yellow,
                      v_x=self.next_vel.x, v_y=self.next_vel.y * inverter, v_theta=self.angle_vel)

    def reset(self):
        self.next_vel = Point(0, 0)
        self.angle_vel = 0

    def decision(self):
        raise NotImplementedError()
    
    def post_decision(self):
        raise NotImplementedError()
    
    def set_vel(self, vel: Point):
        self.next_vel = vel
    
    def set_angle_vel(self, angle_vel: float):
        self.angle_vel = angle_vel
