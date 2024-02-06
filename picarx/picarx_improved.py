import time
import os
import math
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic
    from robot_hat.utils import reset_mcu, run_command
except ImportError:
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic
    from sim_robot_hat.utils import reset_mcu, run_command
import logging
import atexit
import broadcast
import concurrent.futures

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,
datefmt="%H:%M:%S")

logging.getLogger().setLevel(logging.DEBUG)

#from logdecorator import log_on_start, log_on_end, log_on_error

reset_mcu()
time.sleep(0.2)

def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Sensing(object):
    CONFIG = '/opt/picar-x/picar-x.conf'
    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    def __init__(self, 
                grayscale_pins:list=['A0', 'A1', 'A2'],
                config:str=CONFIG,
                ):

          # reset robot_hat
        reset_mcu()
        time.sleep(0.2)
    
    # --------- config_flie ---------
        self.config_flie = fileDB(config, 777, os.getlogin())

    # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())
    
    def producer(self, bus, delay):
    
        while True:
            # Grey scale sensor data production
            sensor_data = self.get_grayscale_data() 
            logging.debug(sensor_data)
            bus.write(sensor_data)
            time.sleep(delay)
    
    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

class Intrepet(object):
    CONFIG = '/opt/picar-x/picar-x.conf'
    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]
    EDGE_THRESHOLD = 0.009  # Threshold for detecting a sharp change

    def __init__(self, 
                config:str=CONFIG,
                ):

          # reset robot_hat
        reset_mcu()
        time.sleep(0.2)

        # --------- config_flie ---------
        self.config_flie = fileDB(config, 777, os.getlogin())

        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
    #polarity = 0 dark on light, polarity = 1 light on dark
    def process_sensor_data(self, sensor_values, polarity, target_is_darker=True):
        p = 0
        if polarity == 0:
            p = 1
        else :
            p=-1
        #Find Average of the sensor values
        l = len(sensor_values)
        s = sum(sensor_values)
        avg=s/l
        #print(avg)

        #Normalize the sensor values
        if avg != 0:
            normalized_values = [sensor_values / avg for sensor_values in sensor_values]
        else:
            normalized_values = [0 for _ in sensor_values]  
        

        total_norm = sum(normalized_values)
        norm_val_avg = total_norm/3
        #differnce betwen first and last sensor
        sensor_range = normalized_values[0]-normalized_values[2]
        #differences of adjacent values in a list
        differences = [normalized_values[i+1] - normalized_values[i] for i in range(len(normalized_values) - 1)]

        avg_diff = (differences[0]+differences[1])/2
        if avg_diff != 0:
            norm_diff = [differences / avg_diff for differences in differences]
        else:
        # Handle the case where avg_diff is zero
        # For example, you might set norm_diff to a list of zeros of the same length as differences
            norm_diff = [0 for _ in differences]
        if norm_val_avg != 0:
            norm_avg_diff = [differences / norm_val_avg for differences in differences]
        else:
            norm_avg_diff = [0 for _ in differences]  # Example: setting all elements to 0
        

        # Detecting edge and its position
        position = 0
        
        if sensor_range < -self.EDGE_THRESHOLD:
        # Check the first element for LEFT or SLIGHT_LEFT
            if abs(norm_avg_diff[0]) > self.EDGE_THRESHOLD:
                position  = p*1
            if abs(norm_avg_diff[0]) < self.EDGE_THRESHOLD:
                position= p*0.5
        if sensor_range > self.EDGE_THRESHOLD:
        # Check the second element for RIGHT or SLIGHT_RIGHT
            if abs(norm_avg_diff[1]) > self.EDGE_THRESHOLD:
                position= p*-1
            if abs(norm_avg_diff[1]) < self.EDGE_THRESHOLD:
                position= p*-0.5

        return position

    def consumer_producer(self, input_bus, output_bus, delay):
    
        while True:
            # Read data from input bus
            data = input_bus.read()
            if data is not None:
                # Process the data
                processed_data = self.process_sensor_data(data,0) 
                logging.debug(processed_data)
                # Write the processed data to the output bus
                output_bus.write(processed_data)
            time.sleep(delay)

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)    
    
    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P12', 'P13'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # reset robot_hat
        reset_mcu()
        time.sleep(0.2)

        # --------- config_flie ---------
        self.config_flie = fileDB(config, 777, os.getlogin())

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        
        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- ultrasonic init ---------
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))

        #Stop at Program Exit
        atexit.register(self.stop)
        
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)

        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def calculate_wheel_speeds(self, base_speed, angle):
        angle_rad = math.radians(angle)
        speed_modifier = math.sin(angle_rad)

        left_speed = base_speed * (1 - speed_modifier)
        right_speed = base_speed * (1 + speed_modifier)

        return left_speed, right_speed

    def backward(self, speed):
        current_angle = self.dir_current_angle
        left_speed, right_speed = self.calculate_wheel_speeds(speed, current_angle)

        self.set_motor_speed(1, -left_speed)
        self.set_motor_speed(2, right_speed)

    def forward(self, speed):
        current_angle = self.dir_current_angle
        left_speed, right_speed = self.calculate_wheel_speeds(speed, current_angle)

        self.set_motor_speed(1, left_speed)
        self.set_motor_speed(2, -right_speed)                

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()

#Forward and backwards in straigt lines with different angles 0 = forward 1 = backward
    def f_b(self, direction, angle, speed, duration):
        a = angle
        d = direction
        s = speed
        t = duration
        self.set_dir_servo_angle(a)

        if (d == 0):
            self.forward(s)
            time.sleep(t)
            self.stop()
        elif (d == 1):
            self.backward(s)
            time.sleep(t)
            self.stop()
        else:
            self.stop()

#Parallel Paking Left and Right 0 = right parking 1 = left parking
    def parallelParking(self, direction):
        d = direction

        if (d==0):
            self.f_b(1, 0, 50, 1)
            self.f_b(1, 30, 50, 1)
            self.f_b(1, -30, 50, 1)
            self.f_b(0, 0, 50, 0.5)
        elif(d==1):
            self.f_b(1, 0, 50, 1)
            self.f_b(1, -30, 50, 1)
            self.f_b(1, 30, 50, 1)
            self.f_b(0, 0, 50, 0.5)
        else:
            self.stop()

#K-turning duration = duration of stop 
    def Kturn(self, duration):
        d = duration
        self.f_b(1, 30, 50, 1.3)
        self.f_b(1, 0, 0, d)
        self.f_b(0, -30, 50, 1.1)

class Controller(Picarx):
    
    def __init__(self):
        super().__init__() 

    def steering_angle(self, position):
        MAX_ANGLE = 25
        MIN_ANGLE = 15
        angle = 0
        if position == 1:
            self.set_dir_servo_angle(-MAX_ANGLE)
            angle = -MAX_ANGLE
            logging.debug(angle)
        elif position == 0.5:
            self.set_dir_servo_angle(-MIN_ANGLE)
            logging.debug(angle)
            angle = -MIN_ANGLE
        elif position == -1:
            self.set_dir_servo_angle(MAX_ANGLE)
            logging.debug(angle)
            angle = MAX_ANGLE
        elif position == -0.5:
            self.set_dir_servo_angle(MIN_ANGLE)
            logging.debug(angle)
            angle = MIN_ANGLE
        elif position == 0:
            self.set_dir_servo_angle(0)
            logging.debug(angle)
            angle = 0
    
    def consumer(self, bus, delay):

        while True:
            # Read processed data from bus
            data = bus.read()
            if data is not None:
                # Act on the data
                self.steering_angle(data)
            time.sleep(delay)


if __name__ == "__main__":
    s = Sensing()
    i = Intrepet()
    c = Controller()
    sensor_values_bus = broadcast.broadcast()
    interpreter_bus = broadcast.broadcast()

    # Set delay times for each function
    sensor_delay = 0.01
    interpreter_delay = 0.02
    control_delay = 0.03
    
    # Set up and run the functions concurrently
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        # Submit tasks to the executor
        eSensor = executor.submit(s.producer, sensor_values_bus, sensor_delay)
        eInterpreter =  executor.submit(i.consumer_producer, sensor_values_bus, interpreter_bus, interpreter_delay)
        eControl = executor.submit(c.consumer, interpreter_bus, control_delay)
        c.forward(30)
        try:
            # This will raise any exceptions caught by the executor
            sensor_result = eSensor.result()
            interpreter_result = eInterpreter.result()
            control_result = eControl.result()
        except Exception as e:
            print(f"An error occurred: {e}")

