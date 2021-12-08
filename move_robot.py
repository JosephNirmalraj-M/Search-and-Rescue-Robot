#!/usr/bin/env python3
import time

from ev3dev2.motor import OUTPUT_B, OUTPUT_C

from robot import Robot

ROBOT_LENGTH = 130.8
MOTOR_PORTS = {"wheel_left": OUTPUT_C, "wheel_right": OUTPUT_B}

robot = Robot(ROBOT_LENGTH, MOTOR_PORTS)

robot.move.mdiff.odometry_start()

robot.move.go_to_coords(150, 0, 1000)
time.sleep(3)
robot.move.slow_down(50, 0, 1000)
time.sleep(3)
robot.move.stop()

# robot.move.go_straight(200, 1000, wait_until_not_moving=False)
# print(robot.move.dist_thread.is_alive())
# time.sleep(3)
# print(robot.move.dist_thread.is_alive())
# robot.move.stop_motors()
# print(robot.move.dist_thread.is_alive())
# time.sleep(5)
# print(robot.move.dist_thread.is_alive())
# robot.move.go_straight(200, 1000, wait_until_not_moving=True)
# print(robot.move.dist_thread.is_alive())
# time.sleep(5)
# print(robot.move.dist_thread.is_alive())

print("BBB")
print("BBB")
print("BBB")
time.sleep(100)

# robot.movediff.turn_right(30, 10*360)

# left_motor = LargeMotor(OUTPUT_C)
# right_motor = LargeMotor(OUTPUT_B)
#
# for i in range(3):
#     left_motor.reset()
#     right_motor.reset()
#     time.sleep(1)
#
#     left_motor.speed_sp = 100
#     right_motor.speed_sp = 100
#
#     print(left_motor.position)
#     print(right_motor.position)
#     time.sleep(1)
#
#     left_motor.run_forever()
#     right_motor.run_forever()
#     # time.sleep(1)
#
#     degrees = 100 / 176 * 360  # = 204
#     dist = 0
#     while dist < degrees:
#         dist = left_motor.position
#
#     left_motor.stop()
#     right_motor.stop()
#
#     print(left_motor.position)
#     print(right_motor.position)
#
# time.sleep(1)
