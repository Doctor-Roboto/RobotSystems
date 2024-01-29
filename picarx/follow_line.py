#!/usr/bin/python3

import time
from sensing import Sensors
from controller import Controller
from interpret import Interpreter
from picarx_improved import Picarx


def follow_line():
    sensor = Sensors()
    input("Press enter to calibrate grayscale, make sure all sensors are on white")

    sensor.calibrate_grayscale()

    # setup car things
    interpreter = Interpreter()
    car = Picarx()
    controller = Controller(car)

    input("Press enter to start")

    while(True):
        values = sensor.read()
        print(values)
        print([a/b for a,b in zip(values,sensor.grayscale_cal_values)])
        controller.control(interpreter.processing(values,sensor.grayscale_cal_values))
        car.forward(30)
        time.sleep(0.1)
        #car.set_dir_servo_angle(-20)


if __name__ == "__main__":
    follow_line()