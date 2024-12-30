
import math
import numpy as np
from rsoccer_gym.Entities import Robot
from utils.Point import Point
from utils.Geometry import Geometry

class Navigation:

    @staticmethod
    def goToPoint(robot: Robot, target: Point):
        reversed = False

        robot_angle = np.deg2rad(robot.theta)
        angle_to_target = math.atan2(target.y - robot.y, target.x - robot.x)
        error = Geometry.smallest_angle_diff(angle_to_target, robot_angle)

        if abs(error) > math.pi / 2.0 + math.pi / 20.0:
            reversed = True
            robot_angle = Geometry.normalize_angle(robot_angle + math.pi)
            error = Geometry.smallest_angle_diff(angle_to_target, robot_angle)

        motor_speed = (20 * error)

        base_speed = 20

        motor_speed = min(max(motor_speed, -base_speed), base_speed)

        if motor_speed > 0:
            left_motor_speed = base_speed
            right_motor_speed = base_speed - motor_speed
        else:
            left_motor_speed = base_speed + motor_speed
            right_motor_speed = base_speed

        if reversed:
            if motor_speed > 0:
                left_motor_speed = -base_speed + motor_speed
                right_motor_speed = -base_speed
            else:
                left_motor_speed = -base_speed
                right_motor_speed = -base_speed - motor_speed

        return left_motor_speed, right_motor_speed
