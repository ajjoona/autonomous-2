"# autonomous-2" 
# Using RaspberryPi 4B and Aduino, make an autonomous vehicle
details refer to _presentation.pdf_

## 1. Problem

Here is the track and obstacles to address.
![image](https://github.com/user-attachments/assets/cb02fab8-4cb1-421f-8869-692fa53a4575)

#### 1. start point
#### 2. tunnel
#### 3. static obstacles (car)
#### 4. dynamic obstacles (human)
#### 5. parking
#### 6. traffic lights



## 2. Solution

### 1) Lane Detection
![image](https://github.com/user-attachments/assets/6290ce9b-50cc-4db1-ad53-d22562d25d3a)
By performing socket communication only when a stop line is recognized (traffic light) or the distance is recognized by an ultrasonic sensor (car/person), speed reduction due to unnecessary communication is prevented.

Canny edge detection method and sliding window tracking technology using openCV were utilized.
To command direction information to the motor, we extracted deviation using x-coordinate values of the line property.
Through several test driving data, a function was derived using linear interpolation between the deviation value and the angle of the servomotor.

The is_dashed property was added to the Line class to determine whether the detected lane was a solid line or a dashed line. The number of empty windows is counted based on the number of pixels within the window, and when two or more out of 6 windows are empty, it is considered a dashed line. Through this, lane changes were made possible regardless of the number and location of vehicle obstacles.


### 2) Object Detection
![image](https://github.com/user-attachments/assets/7c77d250-b135-4eef-bf8c-e24d03a1c3ce)

An object recognition artificial intelligence model was used to detect the colors of people, cars, signs, and traffic lights. Additionally, color detection was performed using openCV to prepare for cases where traffic lights could not be detected through object recognition. This is because the initial traffic light intensity was low and the accuracy of object recognition was low.
A custom dataset was trained using the YOLOv10 model. The custom dataset consisted of test driving data similar to the actual driving environment. Labeling and image preprocessing were performed in Roboflow, and considering that the angle of the obstacle may look different depending on the vehicle's direction of movement, horizontal shear was added to the augmentations to increase the diversity of the dataset. When training the YOLOv10 model, hyperparameter values ​​were adjusted to focus on detecting fast-moving and partially visible objects. 
(Here's the link for training YOLOv10 in Colab: https://colab.research.google.com/drive/1HrctzptCwxvjrN_cVvCVcRGWvTKBJpng#scrollTo=y-lhbe5CPwYV)

By using threads, video streaming and object detection can be performed simultaneously without interfering with each other, minimizing delay. Because the video_stream() function runs in an independent thread, the main loop can continue to perform object detection and processing without delay in video streaming.


### 3) Illuminance Sensor
An illumination sensor was used to turn on the LED when the surroundings became dark.


### 4) ROI settings
Three ROIs were set up: for parking line detection, for stop line detection, and for lane detection. The ROI for parking line detection is set to the right section of the lane, and when a horizontal line is detected, the parking code is executed. The ROI for stop line detection is set to the center of the lane, and the vehicle stops temporarily when a horizontal line is detected.

