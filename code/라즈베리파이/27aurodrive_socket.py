### 화면 출력 부분 삭제한 버전 / process_frame 에서 angle까지 계산 / 주차코드 speed=38로 고정 / find_red,find_green 삭제 / red 탐지하면 종료

import numpy as np
import cv2
import warnings
warnings.filterwarnings('ignore')
import serial
import time
import RPi.GPIO as GPIO
import socket

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # 포트는 실제 연결된 포트에 맞게 설정
time.sleep(2) # 아두이노 초기화 대기

TRIG1 = 14
ECHO1 = 15

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)

# speed = 38
speed = 100
dir = 0
saveangle = 90

HOST = '192.168.2.2'  # 라즈베리 파이의 IP 주소
PORT = 12345

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen()
print("Waiting for a connection...")
client_socket, client_address = server_socket.accept()
print(f"Connected by {client_address}")


def measure_distance(TRIG, ECHO):
    # GPIO.output(TRIG, False)
    # time.sleep(0.1)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance
  
def set_servo_angle(angle):
    command = f"S{angle}\n"
    ser.write(command.encode())

def set_motors_speed(speed):
    command = f"D{speed}\n"
    ser.write(command.encode())

cap2 = cv2.VideoCapture('http://192.168.2.2:8081/')
cap = cv2.VideoCapture(0)
# cap2 = cv2.VideoCapture(1)


src_points = np.array([
    [102, 95],   # 왼쪽 위
    [215, 95],  # 오른쪽 위
    [320, 195],  # 오른쪽 아래
    [0, 195]  # 왼쪽 아래
], dtype=np.float32)

class Line:
    def __init__(self):
        self.prevx = []  
        self.current_fit = None  
        self.startx = None
        self.endx = None
        self.allx = []
        self.ally = []
        self.is_dashed = False  # 차선 상태: 점선인지 실선인지

# left_line = Line()
# right_line = Line()

Parking = False
Stop = False

k_values = [150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40]
j_values = [-170, -150, -120, -100, -80, -20, 5, 30, 60, 120, 140, 160]

def set_roi(frame, x, y, w, h):
    roi = frame[y:y+h, x:x+w]
    return roi

def warp_image(image, src_points):
    width, height = 360, 120
    dst_points = np.float32([[100, 0], [260, 0], [260, 120], [100, 120]])

    M = cv2.getPerspectiveTransform(src_points, dst_points)
    warped_image = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_LINEAR)
    
    return warped_image, M

def canny(img, thresh1=100, thresh2=200):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, thresh1, thresh2)
    return edges

def dilate(img, knl=(5, 1), iter=1): 
    kernel = np.ones(knl, np.uint8)
    dilated_img = cv2.dilate(img, kernel, iterations=iter)
    return dilated_img
 
def update_line(line, line_fit, plotx, ploty):
    if len(line.prevx) > 5:
        avg_line = np.mean(line.prevx, axis=0)
        avg_fit = np.polyfit(ploty, avg_line, 2)
        fit_plotx = avg_fit[0] * ploty ** 2 + avg_fit[1] * ploty + avg_fit[2]
        line.current_fit = avg_fit
        line.allx, line.ally = fit_plotx, ploty
    else:
        line.current_fit = line_fit
        line.allx, line.ally = plotx, ploty
    line.startx, line.endx = line.allx[-1], line.allx[0]
       
def sliding_window_with_previous_info(img, nwindows=6, margin=25, minpix=5, dashed_threshold=2):  # 후속 프레임의 차선 검출 및 추적
    histogram = np.sum(img[int(img.shape[0] // 2):, :], axis=0)
    midpoint = np.intc(histogram.shape[0] // 2)

    base_leftx = left_line.current_fit[2] if left_line.current_fit is not None else np.argmax(histogram[:midpoint])
    base_rightx = right_line.current_fit[2] if right_line.current_fit is not None else np.argmax(histogram[midpoint:]) + midpoint
    
    window_height = np.intc(img.shape[0] / nwindows)
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    left_lane_inds = []
    right_lane_inds = []
    blank_windows_left = 0
    blank_windows_right = 0

    for window in range(nwindows):
        win_y_low = img.shape[0] - (window + 1) * window_height
        win_y_high = img.shape[0] - window * window_height

        win_leftx_min = np.intc(base_leftx - margin)
        win_leftx_max = np.intc(base_leftx + margin)
        win_rightx_min = np.intc(base_rightx - margin)
        win_rightx_max = np.intc(base_rightx + margin)

        left_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high) & (nonzerox >= win_leftx_min) & (nonzerox <= win_leftx_max)).nonzero()[0]
        right_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high) & (nonzerox >= win_rightx_min) & (nonzerox <= win_rightx_max)).nonzero()[0]
        
        left_lane_inds.append(left_window_inds)
        right_lane_inds.append(right_window_inds)

        if len(left_window_inds) > minpix:
            base_leftx = np.intc(np.mean(nonzerox[left_window_inds]))
        else:
            blank_windows_left += 1
        if len(right_window_inds) > minpix:
            base_rightx = np.intc(np.mean(nonzerox[right_window_inds]))
        else:
            blank_windows_right += 1
    
    left_line.is_dashed = blank_windows_left >= dashed_threshold
    right_line.is_dashed = blank_windows_right >= dashed_threshold

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx, lefty = nonzerox[left_lane_inds], nonzeroy[left_lane_inds]
    rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]

    if len(lefty) > 0 and len(leftx) > 0:
        left_fit = np.polyfit(lefty, leftx, 2)
    else:
        if len(righty) > 0 and len(rightx) > 0:
            right_fit = np.polyfit(righty, rightx, 2)
            left_fit = right_fit - np.array([0, 0, 135])  # 차선 폭 픽셀만큼 지정
        else:
            left_fit = left_line.current_fit if left_line.current_fit is not None else np.array([0, 0, base_leftx])

    if len(righty) > 0 and len(rightx) > 0:
        right_fit = np.polyfit(righty, rightx, 2)
    else:
        if len(lefty) > 0 and len(leftx) > 0:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = left_fit + np.array([0, 0, 135])  # 차선 폭 픽셀만큼 지정
        else:
            right_fit = right_line.current_fit if right_line.current_fit is not None else np.array([0, 0, base_rightx])

    left_line.current_fit = left_fit
    right_line.current_fit = right_fit

    ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
    left_plotx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_plotx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    left_line.prevx.append(left_plotx)
    right_line.prevx.append(right_plotx)

    update_line(left_line, left_fit, left_plotx, ploty)
    update_line(right_line, right_fit, right_plotx, ploty)

    return left_line, right_line

def calculate_angle(jamangle):
    global saveangle
    try:
        jangle = None

        for i in range(len(j_values) - 1):
            if j_values[i] <= jamangle <= j_values[i + 1]:
                jangle = int((k_values[i + 1] - k_values[i]) / (j_values[i + 1] - j_values[i]) * (jamangle - j_values[i]) + k_values[i])
                break
        if jangle is None:
            if jamangle <= j_values[0]:
                jangle = int((k_values[1] - k_values[0]) / (j_values[1] - j_values[0]) * (jamangle - j_values[0]) + k_values[0])
            elif jamangle > j_values[-1]:
                jangle = int((k_values[-1] - k_values[-2]) / (j_values[-1] - j_values[-2]) * (jamangle - j_values[-2]) + k_values[-2])

        if jangle is not None and 15 <= jangle <= 165 :
            saveangle = jangle
            return jangle
        else:
            print('outline')
            return saveangle
    except Exception as e:
        print('outline')
        print(e)
        return saveangle

def process_frame(frame, left_line, right_line):
    left_line, right_line = sliding_window_with_previous_info(frame)
    global dir
    if left_line.is_dashed:
        dir = 1
    if right_line.is_dashed:
        dir = 2

    center_lane = (right_line.endx + left_line.endx) / 2
    lane_width = right_line.endx - left_line.endx
    center_car = frame.shape[1] / 2
    deviation_percent = (center_car - center_lane) / (lane_width / 2) * 100
    
    angle = calculate_angle(deviation_percent)

    return left_line, right_line, angle

def detect_horizon(img):  # 주차선 감지
    global Parking
    if Parking:
        return Parking
    
    # cv2.imshow('warped frame for parking', img)
    lines = cv2.HoughLinesP(img, 1, np.pi / 180, threshold=100, minLineLength=50, maxLineGap=10)
    
    if lines is not None:
        count = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180

            if 170 <= angle or angle <= 10:
                count += 1     
        
        if count >= 1:
            print("parkingline")
            parking_code()
            Parking = True

    return Parking

def parking_code():
    set_motors_speed(38)
    time.sleep(0.1)
    
    set_motors_speed(38)
    time.sleep(0.3)
    
    set_motors_speed(0)
    time.sleep(0.2)
    
    set_servo_angle(170)
    time.sleep(0.2)
    
    set_motors_speed(38)
    time.sleep(1.0)  # 필요에 따라 조정 가능
    
    set_motors_speed(0)
    time.sleep(3)
    
    set_servo_angle(170)
    
    set_motors_speed(-38)
    time.sleep(1.0)  # 필요에 따라 조정 가능
    
    set_motors_speed(0)
    time.sleep(0.5)
    
    set_servo_angle(90)
    time.sleep(0.5)
    
    set_motors_speed(38)
    time.sleep(0.4)
                
def detect_stopline(img):  # 정지선 감지
    global Stop
    if Stop:
        return Stop
    
    # cv2.imshow('warped frame for stopline', img)
    lines = cv2.HoughLinesP(img, 1, np.pi / 180, threshold=100, minLineLength=30, maxLineGap=10)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            angle = angle if angle >= 0 else angle + 180
            if 170 <= angle or angle <= 10:
                set_motors_speed(0)
                print("stopline")
                time.sleep(1)
                Stop = True

    return Stop
                

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # (320,240)으로 맞추기
    frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
    canny_img = canny(frame)
    warp_img, _ = warp_image(canny_img, src_points)  # (360,120)
    warp_img = dilate(warp_img)

    lane_roi = set_roi(warp_img, 0, 40, 360, 80)
    left_line, right_line = Line(), Line()
    left_line, right_line, angle = process_frame(lane_roi, left_line, right_line)

    if not Parking:
        park_roi = set_roi(warp_img, 200, 60, 160, 60)
        detect_horizon(park_roi)
    if Parking:
        speed = 10

    if not Stop:
        stop_roi = set_roi(warp_img, 120, 50, 120, 60)
        detect_stopline(stop_roi)
    if Stop:
        while True:
            data = client_socket.recv(1024).decode()
            if data:
                print(f"Received command: {data}")
                if data == "green":
                    for _ in range(80):
                        # data = client_socket.recv(1024).decode()
                        # if data == "red":
                        #     break
                        
                        ret, frame = cap.read()
                        frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
                        canny_img = canny(frame)
                        warp_img, _ = warp_image(canny_img, src_points)  # (360,120)
                        warp_img = dilate(warp_img)

                        lane_roi = set_roi(warp_img, 0, 40, 360, 80)
                        left_line, right_line = Line(), Line()
                        left_line, right_line, angle = process_frame(lane_roi, left_line, right_line)
                        
                        set_motors_speed(20)
                        set_servo_angle(angle)

                    set_motors_speed(0)
                    exit()
                else:
                    set_motors_speed(0)
                    time.sleep(0.1)

    try:
        distance1 = measure_distance(14, 15) #전방 센서
        # print(distance1)
        if distance1 > 30:
            set_servo_angle(angle)
            set_motors_speed(speed)
        elif 5 < distance1 < 30:
            set_motors_speed(0)
            try:
                data = client_socket.recv(1024).decode()
                if data:
                    print(f"Received command: {data}")
                    if data == "car":
                        if dir == 2:
                            print("go_right")
                            set_servo_angle(160)
                            set_motors_speed(speed)
                            time.sleep(0.3) 
                        elif dir == 1:
                            print("go_left")
                            set_servo_angle(15)
                            set_motors_speed(speed)
                            time.sleep(0.3) 
                    elif data == "person":
                        set_motors_speed(0)
                        time.sleep(1)
                        speed = 20
            except socket.error as e:
                print(f"Socket error: {e}")
                break
        else:
            set_motors_speed(0)
            time.sleep(1) 

    except KeyboardInterrupt:
        set_motors_speed(0)
        GPIO.cleanup()
    cv2.waitKey(0)

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break


cap.release()
cv2.destroyAllWindows()
