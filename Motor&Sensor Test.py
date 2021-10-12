#!/usr/bin/env python3

from time import sleep

from ev3dev2.motor import OUTPUT_A, LargeMotor
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sound import Sound


class Robot(object):
    def __init__(self, motor_port=OUTPUT_A):
        self.motor = LargeMotor(motor_port)
        self.speaker = Sound()
        self.color_sensor = ColorSensor()

    def main(self):
        """
        Execute a Motor-Sensor loop:
        1. Detection of a color with the color sensor
        2. The speaker says the name of the color detected
        3. If the color detected is "red", the motor spins for 1 second
        """
        while True:
            detected_color = self.color_sensor.COLORS[self.color_sensor.color]

            self.speaker.speak(detected_color)

            if detected_color == "Red":
                self.motor.on(30)
                sleep(1)
                self.motor.stop()

            sleep(1)


if __name__ == '__main__':
    robot = Robot()  # Initialisation of the robot
    robot.main()  # Execution of the Motor-Sensor loop
