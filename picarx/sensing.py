#!/usr/bin/python3

import time
import statistics
try:
    from robot_hat import ADC
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions ")
    from sim_robot_hat import *

class Sensors(object):
    def __init__(self):
        # grayscale sensors
        self.chn0 = ADC('A0')
        self.chn1 = ADC('A1')
        self.chn2 = ADC('A2')
        self.grayscale_cal_values = []

    def read(self):
        adc_value_list = []
        adc_value_list.append(self.chn0.read())
        adc_value_list.append(self.chn1.read())
        adc_value_list.append(self.chn2.read())
        return adc_value_list

    def calibrate_grayscale(self):
        x = 0
        ch0 = []
        ch1 = []
        ch2 = []
        while (x < 10):
            time.sleep(0.1)
            values = self.read()
            ch0.append(values[0])
            ch1.append(values[1])
            ch2.append(values[2])
            x = x + 1
        self.grayscale_cal_values.append(statistics.mean(ch0))
        self.grayscale_cal_values.append(statistics.mean(ch1))
        self.grayscale_cal_values.append(statistics.mean(ch2))

if __name__ == "__main__":
    sensor = Sensors()
    x = 0
    while (x < 1000000):
        time.sleep(0.5)
        print(sensor.read())
        x = x + 0.1