# Copyright 2024 Euan Steven
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# -*- encoding: utf-8 -*-
# ============== main.py =================
# Author : Euan Steven
# Email : 23006327@uhi.ac.uk
# Date Created : 06/06/2024
# Date Modified : 06/06/2024
# Version : 1.0
# License : Apache 2.0
# Description : Deters Birds from Garden
# ========================================

# ================ To Do =================
# - Firing System
# - Servo Calibration
# - Bird Detection Model

# ============ Module Imports ============
print("======================================== Bird Scarer ========================================")

# Built-in Modules
import time

# Third Party Modules
from numpy import array, expand_dims
from tensorflow.keras.models import load_model
from gpiozero import MotionSensor, Servo
from picamera2 import PiCamera2

# ============= Main Program =============

def setServoAngle(angle_x, angle_y, servo_x, servo_y):
    servo_x.value = angle_x / 180
    servo_y.value = angle_y / 180

def motionDetection(pir):
    pir.wait_for_motion()
    return True

def cameraCapture(camera):
    image_array = PiCamera2.array(camera, camera.sensor_mode)

    camera.capture_into(image_array)

    image_data = image_array.array / 255.0
    image_data = expand_dims(image_data, 0)

    return image_data

def birdDetection(image_data, model):
    height = 418
    width = 418

    bird_coordinates = []

    predictions = model.predict(image_data)

    for box in predictions[0]:
        confidence = box[4]

        if box[5] == 0 and confidence > 0.5:
            x1, y1, x2, y2 = box[:4] * array([width, height, width, height])
            obj_width = int(x2 - x1)
            obj_height = int(y2 - y1)

            bird_coordinates.append((int(x1), int(y1), obj_width, obj_height))

    return bird_coordinates

def aimingSystem(birdCoordinates, servo_x, servo_y):
    max_angle_x = 180
    max_angle_y = 180

    x, y, image_width, image_height = birdCoordinates[0]

    angle_x = (x / image_width) * max_angle_x
    angle_y = (y / image_height) * max_angle_y

    setServoAngle(angle_x, angle_y, servo_x, servo_y)

def firingSystem():
    pass

def main():
    startTime = time.time()

    pir = MotionSensor(4)

    motion = False
    birdCoordinates = []

    camera = PiCamera2()
    camera.resolution = (416,416)
    camera.sensor_mode = 0

    servo_x = Servo(17)
    servo_y = Servo(18)
    
    model = load_model('./model/bird_model.h5')
    
    while True:
            motion = motionDetection(pir)
            if motion:
                print("Motion Detected at " + str(time.time()) + " seconds")
            
                image_data = cameraCapture(camera)
                print("Image Captured at " + str(time.time()) + " seconds")
                
                birdCoordinates = birdDetection(image_data, model)
                print("Bird Detected at " + str(time.time()) + " seconds")
                
                aimingSystem(birdCoordinates, servo_x, servo_y)
                print("Aiming System Activated at " + str(time.time()) + " seconds")
                
                firingSystem()
                print("Firing System Activated at " + str(time.time()) + " seconds")

                endTime = time.time()
                print("\n [OK] Bird Scarer - Elapsed Time : " + str(endTime - startTime) + " seconds")
                break

if __name__ == "__main__":
    main()