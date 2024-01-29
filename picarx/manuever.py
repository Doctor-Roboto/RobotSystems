#!/usr/bin/python3

from picarx_improved import Picarx
import atexit
import time

class Maneuvering(object):
    def __init__(self):
        self.px = Picarx()
        self.default_speed = 80
        self.default_steering = 20
        self.max_steering = 40
        self.pause = 7
        self.command_wait = 0.25
        #atexit.register(self.cleanup)

    def forward_and_backward_with_steering(self):
        # take input
        valid = False
        forward_steer = input("Input forward steering angle: ")
        while not valid:
            try:
                forward_steer = int(forward_steer)
                valid = True
            except ValueError:
                forward_steer = input("Invalid number. Input forward steering angle: ")
            
        valid = False
        backward_steer = input("Input backward steering angle: ")
        while not valid:
            try: 
                backward_steer = int(backward_steer)
                valid = True
            except ValueError:
                backward_steer = input("Invalid number. Input backward steering angle: ")

        # forward
        self.px.set_dir_servo_angle(forward_steer)
        time.sleep(self.command_wait)
        self.px.forward(self.default_speed)
        time.sleep(self.pause)
        self.px.stop()
        time.sleep(self.command_wait)
        self.px.set_dir_servo_angle(0)
        time.sleep(self.command_wait)

        # backward
        self.px.set_dir_servo_angle(backward_steer)
        time.sleep(self.command_wait)
        self.px.backward(self.default_speed)
        time.sleep(self.pause)
        self.px.stop()
        time.sleep(self.command_wait)
        self.px.set_dir_servo_angle(0)

    def parallel_parking(self):
        # take input
        valid = False
        side = input("Input parking side (left or right): ")
        while not valid:
            if side == "left":
                valid = True
                self.px.stop()
                self.px.set_dir_servo_angle(0)
                self.px.forward(70)
                time.sleep(1.2)
                self.px.stop()

                angleRange = [30,-26]
                self.px.set_dir_servo_angle(angleRange[0])
                time.sleep(0.1)

                self.px.backward(70)
                for i in range(angleRange[0],angleRange[1],-1):
                        self.px.set_dir_servo_angle(i)
                        self.px.backward(70)
                        time.sleep(.032)

                self.px.stop()
                self.px.set_dir_servo_angle(0)
            elif side == "right" :
                valid = True
                self.px.stop()
                self.px.set_dir_servo_angle(0)
                self.px.forward(70)
                time.sleep(1.2)
                self.px.stop()

                angleRange = [-26,30]
                self.px.set_dir_servo_angle(angleRange[0])
                time.sleep(0.1)

                self.px.backward(70)
                for i in range(angleRange[0],angleRange[1],1):
                        self.px.set_dir_servo_angle(i)
                        self.px.backward(70)
                        time.sleep(.032)

                self.px.stop()
                self.px.set_dir_servo_angle(0)                
            else:
                side = input("Invalid input. Input parking side (left or right): ")


       
    def k_turn(self):
        # take input
        valid = False
        side = input("Input initial turning side (left or right): ")
        while not valid:
            if side == "left" or side == "right":
                valid = True
            else:
                side = input("Invalid input. Input initial turning side (left or right): ")

        # initial turn
        if side == "left":
            self.px.set_dir_servo_angle(-self.max_steering/2)
        else:
            self.px.set_dir_servo_angle(self.max_steering/2)
        time.sleep(self.command_wait)
        self.px.forward(self.default_speed)
        time.sleep(self.pause*1.2)
        self.px.stop()
        time.sleep(self.command_wait)
        self.px.set_dir_servo_angle(0)
        time.sleep(self.command_wait)

        # backup
        if side == "left":
            self.px.set_dir_servo_angle(self.max_steering/2)
        else:
            self.px.set_dir_servo_angle(-self.max_steering/2)
        time.sleep(self.command_wait)
        self.px.backward(self.default_speed)
        time.sleep(self.pause*1.2)
        self.px.stop()
        time.sleep(self.command_wait)
        self.px.set_dir_servo_angle(0)
        time.sleep(self.command_wait)

        # straighten
        self.px.set_dir_servo_angle(0)
        time.sleep(self.command_wait)
        self.px.forward(self.default_speed)
        time.sleep(self.pause*1.5)
        self.px.stop()
        time.sleep(self.command_wait)
        self.px.set_dir_servo_angle(0)
        time.sleep(self.command_wait)

    def menu(self):
        while True:
            print("Select Something")
            print("1: forward and backward with angle")
            print("2: Parallel Parking")
            print("3: Kturn")
            print("4: exit")

            menu_option = input("Please select a maneuver or q to quit: ")
            if menu_option == "1":
                maneuvering.forward_and_backward_with_steering()
            elif menu_option == "2":
                maneuvering.parallel_parking()
            elif menu_option == "3":
                maneuvering.k_turn()
            elif menu_option == "4":
                return
            else:
                print("Invalid Selection")

    def cleanup(self):
        self.px.set_dir_servo_angle(0)
        self.px.stop

if __name__ == "__main__":
    maneuvering = Maneuvering()
    maneuvering.menu()
