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

# ============ Module Imports ============
print("======================================== Bird Scarer ========================================")

# Built-in Modules
import time
from time import sleep

# Third Party Modules
import cv2
import numpy as np
from tensorflow.keras.models import load_model
from gpiozero import MotionSensor, Servo
from picamera2 import PiCamera2

# ============= Main Program =============

def setServoAngle(angle_x, angle_y):
    servo_x = Servo(17)
    servo_y = Servo(18)

    servo_x.value = angle_x / 180
    servo_y.value = angle_y / 180

    sleep(0.3)

def motionDetection(motion):
    pir = MotionSensor(4)
    motion = False

    while motion is False:
        pir.wait_for_motion()
        motion = True
        print("Motion Detected")

    return motion

def cameraCapture():
    camera = PiCamera2()

    camera.resolution = (416,416)
    camera.sensor_mode = 0
    
    image_array = PiCamera2.array(camera, camera.sensor_mode)

    camera.capture_into(image_array)

    image_data = image_array.array
    image_data = image_data / 255.0
    image_data = np.expand_dims(image_data, 0)

    return image_data

def birdDetection(image_data):
    model = load_model('./model/bird_model.h5')
    image = cv2.imread('./img/image.jpg')
    height, width, _ = image.shape                          # what is this underscore doing here?

    predictions = model.predict(image_data)

    bird_coordinates = []

    for box in predictions[0]:
        confidence = box[4]

        if box[5] == 0 and confidence > 0.5:
            x1, y1, x2, y2 = box[:4] * np.array([width, height, width, height])
            obj_width = int(x2 - x1)
            obj_height = int(y2 - y1)

            bird_coordinates.append((int(x1), int(y1), obj_width, obj_height))

    return bird_coordinates

def aimingSystem(birdCoordinates):
    max_angle_x = 180
    max_angle_y = 180

    x, y, image_width, image_height = birdCoordinates[0]

    angle_x = (x / image_width) * max_angle_x
    angle_y = (y / image_height) * max_angle_y

    setServoAngle(angle_x, angle_y)

def firingSystem():
    pass

def main():
    startTime = time.time()

    motion = False
    birdCoordinates = []
    
    while motion is False:
        motionDetection(motion)
        image_data = cameraCapture()
        birdCoordinates = birdDetection(image_data)
        aimingSystem(birdCoordinates)
        firingSystem()

    endTime = time.time()
    print("\n [OK] Bird Scarer - Elapsed Time : " + str(endTime - startTime) + " seconds")

if __name__ == "__main__":
    main()