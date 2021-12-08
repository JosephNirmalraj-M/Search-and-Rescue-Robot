#!/usr/bin/env python3
import time

from ev3dev2.sound import Sound
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D

from robot import Robot

MOTOR_PORTS = {"wheel_left": OUTPUT_C, "wheel_right": OUTPUT_B, "pickup_left": OUTPUT_D, "pickup_right": OUTPUT_A}
SENSOR_PORTS = {"metal_detector": INPUT_1, "gyro_sensor": INPUT_2, "ultrasonic_sensor": INPUT_4}

ROBOT_LENGTH = 134  # 133.9

DISTANCE_OBJECT = 200
SENSORS_DISTANCE = 75
distance_made = 0

NORMAL_SPEED = 200  # 20
AREA_SPEED = 200  # 15
TURNING_SPEED = 200
OBJECT_DETECTED_SPEED = 50  # 3
AVOID_SPEED = 100  # 10

ODOMETRY = True

sound = Sound()


def pick_object():
    robot.pickup.close_arms()
    time.sleep(2)


def release_object():
    robot.pickup.open_arms()
    time.sleep(2)


def go_back():
    # print("ENTER GO BACK")
    if ODOMETRY:
        robot.move.go_to_coords(AREA_SPEED, 0, 0, wait_until_not_moving=True)
        # robot.move.go_to_coordinates(AREA_SPEED, 0, 0)
    else:
        global distance_made
        print(distance_made)
        # print("TURN LEFT")
        robot.move.turn_left(TURNING_SPEED, 180, use_gyro=True)
        time.sleep(2)
        # print("GO STRAIGHT")
        robot.move.go_straight(AREA_SPEED, distance_made, wait_until_not_moving=True)
    # print("EXIT GO BACK")


def avoid_object(x, y):
    robot.move.go_to_coords(AVOID_SPEED, x, y)


def detect_object(x, y):
    global distance_made
    has_slowed_down = False
    while robot.move.dist_thread.is_alive():
        distance_object = robot.ultrasonic_sensor.value()

        # Object detected at less than 20cm
        if distance_object < DISTANCE_OBJECT:
            # print(robot.metal_detector.analog_read)
            # robot.move.speed_loop = OBJECT_DETECTED_SPEED
            if not has_slowed_down:
                has_slowed_down = True
                robot.move.slow_down(OBJECT_DETECTED_SPEED, x, y)
            # robot.move.change_speed(robot.move.mdiff.left_motor, OBJECT_DETECTED_SPEED)
            # robot.move.change_speed(robot.move.mdiff.right_motor, OBJECT_DETECTED_SPEED)
            if distance_object < SENSORS_DISTANCE:
                # robot.move.stop_motors()
                robot.move.stop()
                # distance_made = robot.move.mdiff.left_motor.position / 360 * robot.move.WHEEL_PERIMETER
                time.sleep(1)

                if not robot.metal_detector.analog_read:
                    sound.speak("Metal object not detected")
                    # sound.beep()
                    avoid_object(x, y)
                    exit()
                else:
                    sound.speak("Metal object detected")
                    # sound.beep()
                    pick_object()
                    go_back()
                    release_object()
                    exit()
        time.sleep(0.05)


def execute_track(track):
    robot.move.mdiff.odometry_start()
    for coords in track:
        robot.move.go_to_coords(AREA_SPEED, coords[0], coords[1])
        time.sleep(0.01)
        detect_object(coords[0], coords[1])

    robot.move.mdiff.odometry_stop()
    print("FINISHED")
    # time.sleep(100)


if __name__ == '__main__':
    robot = Robot(ROBOT_LENGTH, MOTOR_PORTS, SENSOR_PORTS)
    INITIAL_Y = 200
    parallel_track = [(0, 1100),   (200, 1100),  (200, INITIAL_Y),  (400, INITIAL_Y),
                      (400, 1100), (600, 1100),  (600, INITIAL_Y),  (800, INITIAL_Y),
                      (800, 1100), (0, 0)]
    execute_track(parallel_track)
