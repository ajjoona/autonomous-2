![header](https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=18&width=1000&height=200&text=AUTONOMOUS%20ver.2&fontSize=52&fontAlign=39&fontAlignY=32&desc=Using%20RaspberryPi%204B%20and%20Aduino&descSize=22&descAlign=29&descAlignY=52)

### All the details refer to _presentation.pdf_
<br>
<p align="center">
<a href="https://youtu.be/8r7DuOSQ9bw?t=8582"><img src="https://github.com/user-attachments/assets/c49cc4b8-59fa-4a4a-b2e7-d8a0c8994645" width="70%"></a>
</p>

## 0. Overview

|  |  |
| :-: | :-: |
| Period | 2024.07.29. ~ 2024.09.06. (6 weeks) |
| Members | 2 people |
| Language | python 3.9.0 |
| IDE | Visual Studio Code |
| OS | Raspberry Pi OS |


## 1. Problem

Here is the track and obstacles to address.
<p align="center">
<img src="https://github.com/user-attachments/assets/cb02fab8-4cb1-421f-8869-692fa53a4575" width=600>
</p>

<p>1. start point</p>
<p>2. tunnel</p>
<p>3. static obstacles (car)</p>
<p>4. dynamic obstacles (human)</p>
<p>5. parking</p>
<p>6. traffic lights</p>
<br><br>


## 2. Solution

### 1) Lane Detection
<p align="center">
<img src="https://github.com/user-attachments/assets/6290ce9b-50cc-4db1-ad53-d22562d25d3a" width=800>
</p><p>
By performing socket communication only when a stop line is recognized (traffic light) or the distance is recognized by an ultrasonic sensor (car/person), speed reduction due to unnecessary communication is prevented.
</p><p>
Canny edge detection method and sliding window tracking technology using openCV were utilized.
To command direction information to the motor, we extracted deviation using x-coordinate values of the line property.
Through several test driving data, a function was derived using linear interpolation between the deviation value and the angle of the servomotor.
</p><p>
The is_dashed property was added to the Line class to determine whether the detected lane was a solid line or a dashed line. The number of empty windows is counted based on the number of pixels within the window, and when two or more out of 6 windows are empty, it is considered a dashed line. Through this, lane changes were made possible regardless of the number and location of vehicle obstacles.
</p><br>

### 2) Object Detection
<p align="center">
<img src="https://github.com/user-attachments/assets/7c77d250-b135-4eef-bf8c-e24d03a1c3ce" width=400>
</p><p>
An object recognition artificial intelligence model was used to detect the colors of people, cars, signs, and traffic lights. Additionally, color detection was performed using openCV to prepare for cases where traffic lights could not be detected through object recognition. This is because the initial traffic light intensity was low and the accuracy of object recognition was low.
</p><p>
A custom dataset was trained using the YOLOv10 model. The custom dataset consisted of test driving data similar to the actual driving environment. Labeling and image preprocessing were performed in Roboflow, and considering that the angle of the obstacle may look different depending on the vehicle's direction of movement, horizontal shear was added to the augmentations to increase the diversity of the dataset. When training the YOLOv10 model, hyperparameter values ​​were adjusted to focus on detecting fast-moving and partially visible objects. <br>
(Here's the link for training YOLOv10 in Colab: https://colab.research.google.com/drive/1HrctzptCwxvjrN_cVvCVcRGWvTKBJpng#scrollTo=y-lhbe5CPwYV)
</p><p>
By using threads, video streaming and object detection can be performed simultaneously without interfering with each other, minimizing delay. Because the video_stream() function runs in an independent thread, the main loop can continue to perform object detection and processing without delay in video streaming.
</p><br>

### 3) Illuminance Sensor
<p>An illumination sensor was used to turn on the LED when the surroundings became dark.</p>
<br>

### 4) ROI settings
<p>Three ROIs were set up: for parking line detection, for stop line detection, and for lane detection. The ROI for parking line detection is set to the right section of the lane, and when a horizontal line is detected, the parking code is executed. The ROI for stop line detection is set to the center of the lane, and the vehicle stops temporarily when a horizontal line is detected.</p>
<br>
