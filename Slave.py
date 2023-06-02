import sensor
import time
import pyb
import math
from pyb import Timer, Pin, UART
import image, os, tf, uos, gc

uart = UART(1, 115200) # Instantiate UART object

class LaneFollower:
    def __init__(self):


        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        self.clock = time.clock()
        self.red_values = []
        self.green_values = []
        self.blue_values = []
        
        
    def traffic_detect(self, img):
        img_traffic = img.copy()
        red_threshold = 150
        green_threshold = 160
        
        self.circles = img_traffic.find_circles(threshold=1000, x_margin=10, y_margin=10, r_margin=10, r_min=10, r_max=50, r_step=2)
    
        if self.circles:
            for circle in self.circles:
                roi_x = circle.x()
                roi_y = circle.y()
                color = img.get_pixel(roi_x, roi_y)

                # Add color values to the respective lists
                self.red_values.append(color[0])
                self.green_values.append(color[1])
                self.blue_values.append(color[2])

                # Apply moving average filter with window size 5
                window_size = 30
                if len(self.red_values) > window_size:
                    self.red_values = self.red_values[-window_size:]
                if len(self.green_values) > window_size:
                    self.green_values = self.green_values[-window_size:]
                if len(self.blue_values) > window_size:
                    self.blue_values = self.blue_values[-window_size:]

                # Calculate the average values
                red_avg = sum(self.red_values) / len(self.red_values)
                green_avg = sum(self.green_values) / len(self.green_values)
                blue_avg = sum(self.blue_values) / len(self.blue_values)

                # Print or use the average values as needed
                #print("Red average:", red_avg)
                #print("Green average:", green_avg)
                #print("Blue average:", blue_avg)

                if red_avg > red_threshold and green_avg < green_threshold and blue_avg < green_threshold:
                    print("Red traffic")
                    uart.write("R")  # Send 'R' for red traffic light
                    # Process the filtered red blobs within the ROI

                elif red_avg < red_threshold and green_avg > green_threshold and blue_avg < green_threshold:
                    print("Green traffic")
                    uart.write("G")  # Send 'G' for green traffic light
                    # Process the filtered green blobs within the ROI
                else:
                    print("no traffic")   
                    uart.write("G")                  

          

    def run(self):
        while True:
            self.clock.tick()
            img = sensor.snapshot()  # Get a color image
            self.traffic_detect(img)

# Instantiate the LaneFollower object and run the code
lane_follower = LaneFollower()
lane_follower.run()
