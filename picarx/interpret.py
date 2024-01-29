#!/usr/bin/python3

from picarx_improved import Picarx
import numpy as np

class Interpreter(object):
    def __init__(self,sensitivity=0.1,polarity=1):
        self.sensitivity = sensitivity * polarity

    def processing(self, values, cal_values):
        values = [x+1 if x == 0 else x for x in values]
        left, middle, right = values

        left, middle, right = values
        left_cal, middle_cal, right_cal = cal_values
        left = left/left_cal
        middle = middle/middle_cal
        right = right/right_cal

        if abs(right - left) > self.sensitivity: # means we're off the line
            if right > left:
                print("left")
                turn = (left - middle) / (middle+left)
            else:
                print("right") # left > right:
                turn = (middle - right) / (middle+right)
        else:
            print("perfect")# we're perfect
            turn = 0

        return turn