#!/usr/bin/env python3
import time

from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D

from robot import Robot

ROBOT_LENGTH_200 = 130.8  # 133.9
ROBOT_LENGTH_150 = 134
ROBOT_LENGTH = ROBOT_LENGTH_200

# MOTOR_PORTS = {"wheel_left": OUTPUT_C, "wheel_right": OUTPUT_B}
MOTOR_PORTS = {"wheel_left": OUTPUT_C, "wheel_right": OUTPUT_B, "pickup_left": OUTPUT_D, "pickup_right": OUTPUT_A}
SENSOR_PORTS = {"metal_detector": INPUT_1, "gyro_sensor": INPUT_2, "ultrasonic_sensor": INPUT_4}
# robot = Robot(ROBOT_LENGTH, MOTOR_PORTS, SENSOR_PORTS)

#
# class Robot(object):
#     def __init__(self, motor_left_port=OUTPUT_A, motor_right_port=OUTPUT_B):
#         self.tank = MoveTank(motor_left_port, motor_right_port)
#         # self.motor_left = LargeMotor(motor_left_port)
#         # self.motor_right = LargeMotor(motor_right_port)
#         self.degrees = 360*5
#
#     def main(self):
#         """
#         Execute a Motor loop
#         """
#         # self.motor_left.on(100)
#         # self.motor_right.on(100)
#
#         # threading.Thread(target=self.motor_left.on_for_degrees, args=(50, self.degrees)).start()
#         # self.motor_right.on_for_degrees(50, self.degrees)
#         # time.sleep(1)
#         mdiff = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3EducationSetTire, 135.5)
#         mdiff.turn_right(25, 360)
#         # self.tank.on_for_degrees(25, -25, 241*4)
#
#         # self.tank.on_for_degrees(50, 50, 2045)
#         # time.sleep(2)
#         # self.tank.on_for_degrees(50, 0, 437)
#         # # self.tank.on_for_degrees(50, 0, 604)
#         # time.sleep(2)
#         # self.tank.on_for_degrees(50, 50, 2045)
#         # time.sleep(2)
#         # self.tank.on_for_degrees(50, 0, 437)
#         # time.sleep(2)
#         # self.tank.on_for_degrees(50, 50, 2045)
#         # time.sleep(2)
#         # self.tank.on_for_degrees(50, 0, 437)
#         # time.sleep(2)
#         # self.tank.on_for_degrees(50, 50, 2045)
#
#         # (0, 360*2.7) (90, 1062)
#         # thread = threading.Thread(target=self.motor_left.on_for_degrees, args=(50, 1062))
#         # thread.setDaemon(True)
#         # thread.start()
#         # self.motor_right.on_for_degrees(100, 0)
#
#         # time.sleep(1)
#         # threading.Thread(target=self.motor_left.on_for_degrees, args=(50, self.degrees)).start()
#         # self.motor_right.on_for_degrees(50, self.degrees)
#
#         time.sleep(3)
#
#         # print(self.motor_left.position)
#         # print(self.motor_right.position)
#         #
#         # self.motor_right.reset()
#         # self.motor_left.reset()

if __name__ == '__main__':
    robot = Robot(ROBOT_LENGTH, MOTOR_PORTS, SENSOR_PORTS)
    # robot = Robot()  # Initialisation of the robot
    # robot.main()  # Execution of the Motor loop
    # for i in range(1):

    # robot.move.turn_left(200, 360, use_gyro=True)
    robot.move.go_coord_straight(150, 800, 800)
    time.sleep(100)
    # robot.move.turn_right(30, 90)
    # robot.pickup.open_arms()
    # time.sleep(3.5)
    # robot.pickup.close_arms()

# mdiff.odometry_start()
# # Use odometry to drive to specific coordinates
# mdiff.on_to_coordinates(SpeedRPM(40), 300, 300)
# # Use odometry to go back to where we started
# mdiff.on_to_coordinates(SpeedRPM(40), 0, 0)
# # Use odometry to rotate in place to 90 degrees
# mdiff.turn_to_angle(SpeedRPM(40), 90)
# # Disable odometry
# mdiff.odometry_stop()

# from ev3dev2.sensor.lego import TouchSensor
# from ev3dev2.sensor.lego import UltrasonicSensor
# from ev3dev2.led import LED_GROUPS
# from ev3dev2.sound import Sound
#
# # ir = InfraredSensor()
# ts = TouchSensor()
# us = UltrasonicSensor()
# units = us.units
# sound = Sound()
#
# """Keep the IR sensor and US sensor in required modes"""
# # ir.mode = 'IR-PROX'
# us.mode = 'US-DIST-CM'  # Put the US sensor into distance mode. 74.4cm max
# while not ts.value():  # while loop runs as long as the touch sensor button is NOT pressed.When the touch sensor is pressed the loop is exited,
#     # a beep is sounded and the left LED is set to green
#     # distance = ir.value()
#     distance = us.value() / 10  # convert mm to cm
#     print(str(distance) + " " + units)
#
#     if distance < 60:
#         Leds.set_color(LED_GROUPS['LEFT'], Leds.RED)
#     else:
#         Leds.set_color(Leds.LEFT, Leds.GREEN)
#
# sound.beep()
# # Leds.set_color(Leds.LEFT, Leds.GREEN)
#
#
# # straight turn 30 90° left right
