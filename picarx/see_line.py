#!/usr/bin/python3

from matplotlib import pyplot as plt
from time import sleep
import numpy
import math

from cameraSensorClass import PictureTaker
import picarx_improved as pc


try:
    from robot_hat.utils import reset_mcu
except ImportError:
    print('ImportError, using sim_robot_hat')
    from sim_robot_hat.utils import reset_mcu


reset_mcu()
sleep(0.2)

"""
Code modified from https://const-toporov.medium.com/line-following-robot-with-opencv-and-contour-based-approach-417b90f2c298

"""
def getch():
    import termios
    import sys, tty
    def _getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    return _getch()


class sensor():

    def __init__(self, picTaker):
        self.picTaker = picTaker

    def sensor_reading(self, DEBUG=False):
        edge = self.picTaker.takePicture()
        crop = self.picTaker.lastCrop
        
        if DEBUG:
            plt.subplot(211)
            plt.imshow(crop,cmap='gray')
            plt.subplot(212)
            plt.imshow(edge,cmap='gray')

            #Plot images, then pause
            plt.show(block=False)
            plt.pause(0.1)
            plt.close
        
        return edge

# Try identifying angle of line off vert
class interpreter():  
    def __init__(self):
        self.running_avg_angles = [0,0,0]
    
    def process_reading(self, reading):
        # reading is image crop from sensing
        # find an edge
        lower_row = 140
        upper_row = 120
        col = 0
        max_col = 200
        max_row = 150
        white = 50
        
        lower_point = [0,0]
        upper_point = [0,0]
        lower_found = False
        upper_found = False
        
        while (not (lower_found and upper_found)) and col < max_col:
            lower_val = (reading[lower_row][col] + reading[lower_row+1][col] + reading[lower_row-1][col])/3
            upper_val = (reading[upper_row][col] + reading[upper_row+1][col] + reading[upper_row-1][col])/3
            
            if lower_val >= white and not lower_found:
                lower_point = [lower_row, col]
                lower_found = True
                print('found lower point: ', lower_point)
                
            if upper_val >= white and not upper_found:
                upper_point = [upper_row, col]
                upper_found = True
                print('found upper point: ', upper_point)
            
            col += 1
        
        
        # if both points found, determine angle off 0
        # angle = 0 TODO perhaps go straight if no angle found
        if lower_found and upper_found:
            slope = (upper_point[1]-lower_point[1]) / (upper_point[0]-lower_point[0])
            angle = (-1)*math.atan(slope)
            #print('Raw Angle:', angle)
            
            self.running_avg_angles[0] = self.running_avg_angles[1]
            self.running_avg_angles[1] = self.running_avg_angles[2]
            self.running_avg_angles[2] = angle

    def output(self):
        # return average of stored angles
        val = sum(self.running_avg_angles) / len(self.running_avg_angles)
        print('Steering command: ', val)
        return val
    
class controller():

    def __init__(self, scaling_factor=45):
        # take in an argument (with a default value) for the scaling factor
        # between the interpreted offset from the line and the angle by which 
        # to steer
        self.scaling_factor = scaling_factor

    def control(self, line_angle=0.0):
        # take sensed line angle and cars current steering command
        #cmd_angle = int(rel_pos * self.scaling_factor)
        cmd_angle = int(line_angle * self.scaling_factor)
        return cmd_angle

def line_following():
    
    picTaker = PictureTaker()
    car = pc.Picarx()
    s = sensor(picTaker)
    i = interpreter()
    c = controller()

    car.set_cam_tilt_angle(-25)
    car.forward(65)
    sleep(0.5)
    car.forward(40)
    print("Starting... Press x to stop")
    
    old_cmd = 0
    step = 0
    
    while True:        
        read_sensor = s.sensor_reading(DEBUG=True)
        i.process_reading(read_sensor)
        process_val = i.output()
        print('Processed val: ', process_val)
        cmd = c.control(process_val)
        print('Commanded Steering: ', cmd)
        sleep(0.25)
        
        if cmd != old_cmd or step == 5:
            car.set_dir_servo_angle(cmd) 
            
            car.forward(75)
            sleep(0.75)
            car.stop()
            
            old_cmd = cmd
            step = 0
        
        step += 1
        



if __name__ == "__main__":
    line_following()
