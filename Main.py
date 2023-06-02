import sensor
import time
import pyb
import math
from pyb import Timer, Pin, UART
import image, os, tf, uos, gc


class LaneFollower:
    def __init__(self):
        # Camera parameters setup
        self.roi = (0, 45, 160, 75)  # Default ROI

        # Desired lane center
        self.desired_lane_center = 80

        # Motor Setup
        self.DC_MOTOR_FREQ = 1500
        self.SERVO_FREQ = 100

        # Timer Setup
        self.timer2 = pyb.Timer(2, freq=self.DC_MOTOR_FREQ)  # DC Motor
        self.timer4 = pyb.Timer(4, freq=self.SERVO_FREQ)  # Servo

        # Channel setup
        self.dc_motor_channel = self.timer2.channel(1, Timer.PWM, pin=pyb.Pin.board.P6)
        self.servo_channel = self.timer4.channel(3, Timer.PWM, pin=pyb.Pin.board.P9)

        # Initial PWM duty cycle
        self.duty_cycle = 20

        # Camera Setup
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        self.clock = time.clock()

        self.dc_motor_channel.pulse_width_percent(0)
        self.servo_channel.pulse_width(0)

        # PID parameters (now PD parameters)  33 1.15
        self.kp = 33
        self.kd = 1.15
        self.prev_error = 0


        # Servo settings
        self.SERVO_NEUTRAL = 2850
        self.SERVO_MIN = 2200
        self.SERVO_MAX = 3800

        # Initialize last_center with desired_lane_center
        self.last_center = self.desired_lane_center

        # Initialize the last known positions of the lines
        self.last_max_left = 0
        self.last_max_right = sensor.width()
        self.last_max_left_line = 0
        self.last_max_right_line = 0

        # Initialize previous servo command
        self.prev_servo_command = self.SERVO_NEUTRAL

        # Damping factor
        self.DAMPING_FACTOR = 0.5

        # Define the size of the moving average filter
        self.MAF_SIZE = 8

        # Initialize the moving average filter list with the desired_lane_center
        self.maf_list = [self.desired_lane_center for _ in range(self.MAF_SIZE)]

                # Define the size of the moving average filter for deflection angle
        self.MAF_SIZE_ANGLE = 4

        # Initialize the moving average filter list for deflection angle
        self.maf_list_angle = [0 for _ in range(self.MAF_SIZE_ANGLE)]

        #dc motor control flag
        self.tlt = 0
        self.ot = 0
        self.go = 0

        self.break_pin = Pin('P5', Pin.OUT_PP)
        self.accel_pin = Pin('P3', Pin.OUT_PP)
        self.accel_pin.high()
        self.break_pin.low()
        self.uart = UART(1, 115200)  # same parameters as the sender


        self.red_values = []
        self.green_values = []
        self.blue_values = []


    def measure_distance_cm(self):
        trigger_pin = Pin('P7', Pin.OUT_PP)
        echo_pin = Pin('P8', Pin.IN)

        # Send a 10us pulse.
        trigger_pin.high()
        time.sleep_us(10)
        trigger_pin.low()

        while echo_pin.value() == 0:
            pass

        t1 = time.ticks_us()

        while echo_pin.value() == 1:
            pass

        t2 = time.ticks_us()
        return ((t2 - t1) * 0.343) / 2.0

    def control_dc_motor(self, duty_cycle):
        self.dc_motor_channel.pulse_width_percent(duty_cycle)

    def control_servo_motor(self, servo_command):
        self.servo_channel.pulse_width(servo_command)

    def motor_control(self, duty_cycle, tlt, ot):

        if self.ot == 1:
                # Stop the car
                if self.go == 1:
                    self.control_dc_motor(22)
                else:
                    self.control_dc_motor(0)

        elif self.tlt == 1:
                if self.go == 1:
                    self.control_dc_motor(22)
                else:
                    self.control_dc_motor(0)
        else:
            self.control_dc_motor(duty_cycle)



    def detect_lines(self, img):
        img_gray = img.copy()
        img_gray.to_grayscale()

        max_left = None
        max_right = None
        max_left_line = None
        max_right_line = None

        for l in img_gray.find_lines(roi=self.roi, threshold=1550, x_stride=2, y_stride=1):
            if l.theta() != 89 and l.theta() != 0:
                avg_x = (l.x1() + l.x2()) / 2  # average x-coordinate
                if avg_x > (max_left if max_left is not None else 0):
                    max_left = avg_x
                    max_left_line = l
                if avg_x < (max_right if max_right is not None else img.width()):
                    max_right = avg_x
                    max_right_line = l


        # If both lines are detected, calculate the center
        if max_left is not None and max_right is not None:
            lane_center = (max_left + max_right)//2
            #print("Lane Center:", lane_center)
            self.last_center = lane_center  # Update the last center
        else:
            lane_center = self.last_center  # Use the last center

        # Add the current lane_center to the list and calculate the average
        self.maf_list.pop(0)  # Remove the oldest value
        self.maf_list.append(lane_center)  # Add the new value
        lane_center_avg = sum(self.maf_list) / self.MAF_SIZE  # Calculate the average

        if 78 <= lane_center_avg <= 82:
            error = 0
        else:
            error = self.desired_lane_center - lane_center_avg

        # Calculate deflection angle and add to MAF
        if max_left_line and max_right_line:
            left_angle = max_left_line.theta()
            right_angle = max_right_line.theta()
            deflection_angle = (left_angle - right_angle) / 2
            # Add the current deflection angle to the list and calculate the average
            self.maf_list_angle.pop(0)  # Remove the oldest value
            self.maf_list_angle.append(deflection_angle)  # Add the new value
            deflection_angle_avg = sum(self.maf_list_angle) / self.MAF_SIZE_ANGLE  # Calculate the average
        else:
            deflection_angle_avg = sum(self.maf_list_angle) / self.MAF_SIZE_ANGLE  # Use the average of the past values if current value is not available

        # Adjust speed according to deflection angle
        if 0 <= abs(deflection_angle_avg) < 10:
            duty_cycle = self.duty_cycle = 17
        elif 10 <= abs(deflection_angle_avg) < 15:
            duty_cycle = self.duty_cycle = 18
        elif 15 <= abs(deflection_angle_avg) < 30:
            duty_cycle = self.duty_cycle = 18
        else:
            duty_cycle = self.duty_cycle = 17
        return error, max_left_line, max_right_line, duty_cycle


    def calculate_pid_output(self, error):
        proportional = self.kp * error
        derivative = self.kd * (error - self.prev_error)
        return proportional + derivative

    def draw_lines(self, img, max_left_line, max_right_line):
        if max_left_line:
            img.draw_line(max_left_line.line(), color=(255, 0, 0), thickness=4)
        if max_right_line:
            img.draw_line(max_right_line.line(), color=(255, 0, 0), thickness=4)

    def determine_distance(self):
        distance = self.measure_distance_cm()
        if distance < 200:  # If the cone is closer than 30 cm, stop the car
            self.ot = 1
            self.bypass_object()
        else:
            self.ot = 0

    def bypass_object(self):
        # Stop the car

        self.go = 1
        self.accel_pin.low()
        self.break_pin.high()
        self.control_servo_motor(self.SERVO_NEUTRAL)

        pyb.delay(2000)

        self.control_servo_motor(self.SERVO_MIN)

        self.accel_pin.high()
        self.break_pin.low()

        self.motor_control(self.duty_cycle,self.tlt, self.ot)

        pyb.delay(1000)
        self.go = 1

        self.motor_control(self.duty_cycle,self.tlt, self.ot)

    def red_light_stop(self):
        self.go = 0
        self.accel_pin.low()
        self.break_pin.low()


    def reciever(self):
        if self.uart.any():  # if there's any data available
            data = self.uart.read()  # read all data
            if data:
                if b'R' in data:
                    print(self.tlt)
                    self.tlt = 1
                    self.red_light_stop()
                elif b'G' in data:
                    self.tlt = 0
                    self.go = 1
                    self.accel_pin.high()
                    self.motor_control(self.duty_cycle, self.tlt, self.ot)
                else:
                    self.tlt = 0
                    self.go = 1
                    self.accel_pin.high()
                    self.motor_control(self.duty_cycle, self.tlt, self.ot)



    def run(self):
        while True:
            self.clock.tick()
            img = sensor.snapshot()  # Get a color image


            #car functions
            self.determine_distance()
            self.reciever()



            error, max_left_line, max_right_line, duty_cycle = self.detect_lines(img)

            # Calculate PD controller output
            pd_output = self.calculate_pid_output(error)

            # Calculate new servo command based on PD output (clamped to SERVO_MIN and SERVO_MAX)
            new_servo_command = self.SERVO_NEUTRAL + pd_output
            new_servo_command = min(max(int(new_servo_command), self.SERVO_MIN), self.SERVO_MAX)

            # Implement damping
            servo_command = self.DAMPING_FACTOR * self.prev_servo_command + (
                    1 - self.DAMPING_FACTOR) * new_servo_command
            servo_command = int(servo_command)

            # Assign output to servo motor
            self.control_servo_motor(servo_command)

            self.prev_servo_command = servo_command
            self.prev_error = error

            # Draw lines
            self.draw_lines(img, max_left_line, max_right_line)

            img.draw_rectangle(self.roi, color=(0, 255, 0), thickness=2)



# Instantiate the LaneFollower object and run the code
lane_follower = LaneFollower()
lane_follower.run()
