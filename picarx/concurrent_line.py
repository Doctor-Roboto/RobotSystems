#!/usr/bin/env python3
from picarx_improved import Picarx, ADC, logging, constrain
from time import sleep
import numpy as np
from bus import Bus
import concurrent.futures

class Grayscale_Sensor(object):
    LEFT = 0
    """Left Channel"""
    MIDDLE = 1
    """Middle Channel"""
    RIGHT = 2
    """Right Channel"""
    REFERENCE_DEFAULT = [1000]*3

    def __init__(self, pin0: ADC, pin1: ADC, pin2: ADC, reference: int = None):
        """
        Initialize Grayscale Module

        :param pin0: ADC object or int for channel 0
        :type pin0: robot_hat.ADC/int
        :param pin1: ADC object or int for channel 1
        :type pin1: robot_hat.ADC/int
        :param pin2: ADC object or int for channel 2
        :type pin2: robot_hat.ADC/int
        :param reference: reference voltage
        :type reference: 1*3 list, [int, int, int]
        """
        self.pins = (pin0, pin1, pin2)
        for i, pin in enumerate(self.pins):
            if not isinstance(pin, ADC):
                raise TypeError(f"pin{i} must be robot_hat.ADC")
        self._reference = self.REFERENCE_DEFAULT

    def reference(self, ref: list = None) -> list:
        """
        Get Set reference value

        :param ref: reference value, None to get reference value
        :type ref: list
        :return: reference value
        :rtype: list
        """
        if ref is not None:
            if isinstance(ref, list) and len(ref) == 3:
                self._reference = ref
            else:
                raise TypeError("ref parameter must be 1*3 list.")
        return self._reference

    def read_status(self, datas: list = None) -> list:
        """
        Read line status

        :param datas: list of grayscale datas, if None, read from sensor
        :type datas: list
        :return: list of line status, 0 for white, 1 for black
        :rtype: list
        """
        if self._reference == None:
            raise ValueError("Reference value is not set")
        if datas == None:
            datas = self.read()
        return [0 if data > self._reference[i] else 1 for i, data in enumerate(datas)]

    def read(self, channel: int = None) -> list:
        """
        read a channel or all datas

        :param channel: channel to read, leave empty to read all. 0, 1, 2 or Grayscale_Module.LEFT, Grayscale_Module.CENTER, Grayscale_Module.RIGHT 
        :type channel: int/None
        :return: list of grayscale data
        :rtype: list
        """
        if channel == None:
            return [self.pins[i].read() for i in range(3)]
        else:
            return self.pins[channel].read()
        
    def producer(self, bus, delay = .01):
        """ Grab sensor data and publish it at the rate we want """
        while True:
            bus.write(self.read())
            sleep(delay)
        
    

class Interpreter():
    """Interprets the greyscale sensor data and returns where the line is: 
         robot relative to the line as a value on the interval [−1,1], with positive values being to the left of the robot."""
    def __init__(self, light_sensitivity = 25, dark_sensitivity = 20, follow_dark_line = True):
        self.dark = dark_sensitivity
        self.light = light_sensitivity
        self.polarity = follow_dark_line
        self.counter = 0
        
        
    def return_pos(self, greyscale_value = [0, 0, 0]):
        
        # First check if we have a least one value below the thresehold and one value above, or we failed
        min_val = min(greyscale_value)
        max_val = max(greyscale_value)
        if not (min_val < self.dark) or not (max_val > self.light):
            self.counter += 1
            if self.counter > 20: 

                logging.error(f"Min/max thresehold not hit - following failed.")
                exit()
            return 0
        
        self.counter = 0
            
        # Check if the min/max is in the middle
        if self.polarity and greyscale_value.index(min(greyscale_value)) == 1:
            # Dark line, and the darkest is in the middle
            
            # Line towards the center, but we will adjust towards side based on which one is greater
            edges = [greyscale_value[0], greyscale_value[2]]
            ratio = min(edges)/max(edges)
            if greyscale_value[0] > greyscale_value[2]:
                sign = 1
            else: 
                sign = -1
            return sign * constrain(1 - ratio, -.5, .5)
        elif not self.polarity and greyscale_value.index(max(greyscale_value)) == 1:
            # light line, and the brightest is in the middle
            
            # Line towards the center, but we will adjust towards side based on which one is greater
            edges = [greyscale_value[0], greyscale_value[2]]
            ratio = min(edges)/max(edges)
            if greyscale_value[0] > greyscale_value[2]:
                sign = -1
            else: 
                sign = 1
            return sign * constrain(1 - ratio, -.5, .5)
        elif self.polarity and greyscale_value.index(min(greyscale_value)) == 2:
            # Dark line, edge to right
            
            return greyscale_value[1]/greyscale_value[2]
        
        elif self.polarity and greyscale_value.index(min(greyscale_value)) == 0:
            return -greyscale_value[1]/greyscale_value[0]
        elif not self.polarity and greyscale_value.index(max(greyscale_value)) == 2:
            # Dark line, edge to right
            
            return greyscale_value[1]/greyscale_value[2]
        
        elif not self.polarity and greyscale_value.index(max(greyscale_value)) == 0:
            return -greyscale_value[1]/greyscale_value[0]
        
        
    def interpreter(self, bus_greyscale, bus_line_pos, delay = .01):
        """ Grabs data from greyscale bus, processes and outputs to the bus_line_pos """
        try:
            while True:
                greyscale_data = bus_greyscale.read()
                position = self.return_pos(greyscale_data)
                bus_line_pos.write(position)
                sleep(delay)
        except: 
            return
            
class Controller():
    def __init__(self, picar: Picarx, scaling_factor = 15.0):
        self.scale_factor = scaling_factor
        self.picar = picar
        
    def update_angle(self, line_pos = 0.0):
        scaled_val = self.scale_factor * line_pos
        self.picar.set_dir_servo_angle(scaled_val)
        return(scaled_val)
    
    def controller_consumer(self, bus_line_pos, delay = .01):
        try:
            while True:
                
                line_pos = bus_line_pos.read()
                self.update_angle(line_pos)
                logging.debug(f"New pos: {line_pos}")
                sleep(.01)
        except:
            return

def old_control():
    # Set up picar class
    picar = Picarx()
    # Set up greyscale class for reading 
    grey_sensor = Grayscale_Sensor(ADC(Grayscale_Sensor.LEFT), ADC(Grayscale_Sensor.MIDDLE), ADC(Grayscale_Sensor.RIGHT))
    inter = Interpreter(40, 32, False) # was 25, 20
    control = Controller(picar, 40)
    avg_read = 0
    reading = np.zeros((5,3))
    picar.forward(24)
    prev_angle = 0
    try:
        while True:
            # Get three readings
            for i in range(5):
                sensor_read = grey_sensor.read()
                reading[i, :] = sensor_read
                
            avg_reading = list(np.mean(reading, axis=0))
            
            logging.debug(f"Avg reading: {avg_reading}")
            inter_val = inter.return_pos(avg_reading)
            logging.debug(f"Line position: {inter_val}")
            
            control.update_angle((inter_val*1.5+prev_angle*.5)/2)
            prev_angle = inter_val
            sleep(.05)
    except:
        return
    
        
    
        
if __name__ == "__main__":
    picar = Picarx()
    greyscale = Grayscale_Sensor(ADC(Grayscale_Sensor.LEFT), ADC(Grayscale_Sensor.MIDDLE), ADC(Grayscale_Sensor.RIGHT))
    interp = Interpreter(40, 32, False)
    control = Controller(picar, 40)
    
    # Create the buses
    greyscale_bus = Bus()
    line_pos_bus = Bus()
    
    sensor_delay = .05
    interpreter_delay = .05
    controller_delay = .1
    picar.forward(30)
    
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(greyscale.producer, greyscale_bus, sensor_delay)
        sleep(.1)
        eInterpreter = executor.submit(interp.interpreter,greyscale_bus, line_pos_bus, interpreter_delay)
        sleep(.1)
        eController = executor.submit(control.controller_consumer, line_pos_bus, controller_delay)
    picar.stop()
