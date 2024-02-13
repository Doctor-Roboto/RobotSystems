import rossros as rr
import time
from picarx_improved import Picarx

try:
    from robot_hat import ADC, Pin
except ImportError:
    from sim_robot_hat import ADC, Pin

px = Picarx()

""" First Part: Signal reading and processing functions """

class Gray_Sensing():

    def __init__(self):
            self.chn_0 = ADC('A0')
            self.chn_1 = ADC('A1')
            self.chn_2 = ADC('A2')

    def get_grayscale_data(self):
        adc_value_list = []
        adc_value_0 = self.chn_0.read()
        adc_value_1 = self.chn_1.read()
        adc_value_2 = self.chn_2.read()
        #print(f"ADC values: {adc_value_0}, {adc_value_1}, {adc_value_2}")
        adc_value_list.append(adc_value_0)
        adc_value_list.append(adc_value_1)
        adc_value_list.append(adc_value_2)
        return adc_value_list

      
    def read(self):
        return self.get_grayscale_data()

class Gray_Interpreter():
     
    def __init__(self,sensitivity=0.7, polarity=-1):
        self.sensitivity = sensitivity
        self.polarity = polarity
    
    def process_sensor_data(self, readings):
        # Interpret the readings
        avg = sum(readings) / len(readings)
        if self.polarity == 1:
            interpreted_readings = [1 if (reading - avg) > self.sensitivity else 0 for reading in readings]
        else:
            interpreted_readings = [0 if (reading - avg) > self.sensitivity else 1 for reading in readings]

        # Map the interpreted readings to a value
        if interpreted_readings == [0, 1, 0]:
            return 0
        elif interpreted_readings == [0, 1, 1]:
            return 1
        elif interpreted_readings == [0, 0, 1]:
            return 1
        elif interpreted_readings == [1, 1, 0]:
            return -1
        elif interpreted_readings == [1, 0, 0]:
            return -1
        elif interpreted_readings == [1, 1, 1]:
            return 0
        elif interpreted_readings == [0, 0, 0]:
            return 0
        else:
            return None  # Return None if the interpreted readings do not match any known pattern

class Gray_Controller():
    def __init__(self,scaling=1.0):
        self.scaling = scaling

    def control(self, value):
        #print(f"Controller is processing value: {value}")
        if value == 0:
            px.set_dir_servo_angle(0)
        elif value == 1:
            px.set_dir_servo_angle(30*self.scaling)
        elif value == -1:
            px.set_dir_servo_angle(-30*self.scaling)
        # elif value == -0.5:
        #     px.set_dir_servo_angle(-30*self.scaling)
        # elif value == 0.5:
        #     px.set_dir_servo_angle(30*self.scaling)

class Ult_Sensing():
    def __init__(self, timeout=0.02):
        self.trig = Pin('D2')
        self.echo = Pin('D3')
        self.timeout = timeout

    def get_ultrasonic_data(self):
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.00001)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -1
        during = pulse_end - pulse_start
        return during
    
    def read(self):
        return self.get_ultrasonic_data()

class Ult_Interpreting():
    def __init__(self):
        pass

    def interpret(self, data):
        if data == -1:
            return 1000
        cm = round(data * 340 / 2 * 100, 2)
        return cm
    
class Ult_Control():
    def _init_(self):
        pass

    def controller(self, threshold, speed=35):
        if threshold < 10:
            px.forward(0)
            px.stop
        else:
            px.forward(speed)

# class Stop_Motors():
#     def __init__(self):
#         pass

#     def stop(self, signal):
#         if signal == 1:
#             px.stop()
#             px.stopping_motors()

""" Second Part: Create buses for passing data """
gray_sensor = Gray_Sensing()
gray_interpreter = Gray_Interpreter(sensitivity=0.95, polarity=-1) #light line (-1) , dark line (1)
gray_controller= Gray_Controller(scaling=1)

ult_sensor = Ult_Sensing()
ult_interpreter = Ult_Interpreting()
ult_controller = Ult_Control()

# stop_motors = Stop_Motors()

# Initiate data and termination busses
bGray_Sensing = rr.Bus(gray_sensor.read(), "Sensing bus")
bGray_Interpreter = rr.Bus(gray_interpreter.process_sensor_data(gray_sensor.read()), "Interpreter Bus")
bGray_Controller = rr.Bus(gray_controller.control(gray_interpreter.process_sensor_data(gray_sensor.read())), "Controller bus")

bUlt_Sensing = rr.Bus(ult_sensor.read(), "Sensing bus")
bUlt_Interpreting = rr.Bus(ult_interpreter.interpret(ult_sensor.read()), "Interpreter Bus")
bUlt_Controller = rr.Bus(ult_controller.controller(ult_interpreter.interpret(ult_sensor.read())), "Controller bus")

bTerminate = rr.Bus(0, "Termination Bus")

# bStop_Motors = rr.Bus(stop_motors.stop(1), "Stop Motors bus")
 
""" Third Part: Wrap functions into RossROS objects """

# Wrap the sensing greyscale data into a producer
readPins_gray = rr.Producer(
    gray_sensor.get_grayscale_data,  # function that will generate data
    bGray_Sensing,  # output data bus
    0.001,  # delay between data generation cycles
    bTerminate,  # bus to watch for termination signal
    "Read pin data from greyscale module")

# Wrap the multiplier function into a consumer-producer
interpretData_gray = rr.ConsumerProducer(
    gray_interpreter.process_sensor_data,  # function that will process data
    bGray_Sensing,  # input data buses
    bGray_Interpreter,  # output data bus
    0.005,  # delay between data control cycles
    bTerminate,  # bus to watch for termination signal
    "Interpret grayscale data")

# Wrap the multiplier function into a consumer-producer
controlServo_gray = rr.Consumer(
    gray_controller.control,  # function that will process data
    bGray_Interpreter,  # input data buses
    0.01,  # delay between data control cycles
    bTerminate,  # bus to watch for termination signal
    "Move direction servo")

readPins_ult = rr.Producer(
    ult_sensor.get_ultrasonic_data,  # function that will generate data
    bUlt_Sensing,  # output data bus
    0.002,  # delay between data generation cycles
    bTerminate,  # bus to watch for termination signal
    "Read pin data from ultrasonic module")

interpretData_ult = rr.ConsumerProducer(
    ult_interpreter.interpret,  # function that will process data
    bUlt_Sensing,  # input data buses
    bUlt_Interpreting,  # output data bus
    0.008,  # delay between data control cycles
    bTerminate,  # bus to watch for termination signal
    "Interpret ultrasonic data")

controlServo_ult = rr.Consumer(
    ult_controller.controller,  # function that will process data
    bUlt_Interpreting,  # input data buses
    0.02,  # delay between data control cycles
    bTerminate,  # bus to watch for termination signal
    "Move direction servo")

# stopMotors = rr.Consumer(
#     stop_motors.stop,  # function that will process data
#     bTerminate,  # input data buses
#     0.01,  # delay between data control cycles
#     bTerminate,  # bus to watch for termination signal
#     "Stop Motors")


""" Fourth Part: Create RossROS Printer and Timer objects """

# Make a printer that returns the most recent wave and product values
printBuses = rr.Printer(
    (bGray_Sensing, bGray_Interpreter, bTerminate),  # input data buses
    # bMultiplied,      # input data buses
    0.25,  # delay between printing cycles
    bTerminate,  # bus to watch for termination signal
    "Print raw and derived data",  # Name of printer
    "Data bus readings are: ")  # Prefix for output

# Make a timer (a special kind of producer) that turns on the termination
# bus when it triggers
terminationTimer = rr.Timer(
    bTerminate,  # Output data bus
    15,  # Duration
    0.01,  # Delay between checking for termination time
    bTerminate,  # Bus to check for termination signal
    "Termination timer")  # Name of this timer

""" Fifth Part: Concurrent execution """

# Create a list of producer-consumers to execute concurrently
producer_consumer_list = [readPins_gray,
                          interpretData_gray,
                          controlServo_gray,
                          readPins_ult,
                          interpretData_ult,
                          controlServo_ult,
                          printBuses,
                          terminationTimer] # add printBuses, if using printer

# Execute the list of producer-consumers concurrently
rr.runConcurrently(producer_consumer_list)