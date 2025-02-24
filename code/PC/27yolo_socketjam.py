import cv2
import time
import threading
import socket
from ultralytics import YOLO
import numpy as np

# YOLOv8 모델 로드 (ONNX 파일 경로를 설정)
model = YOLO('best (11).pt')  # 여기서 ONNX 파일 경로를 지정

# 전역 변수로 최신 프레임 저장
frame = None
size = 10

# 서버의 IP 주소와 포트 번호 설정
SERVER_IP = '192.168.2.2'  # 라즈베리 파이의 IP 주소
SERVER_PORT = 12345

# 소켓 생성 및 서버 연결
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((SERVER_IP, SERVER_PORT))

def video_stream():
    global frame
    cap = cv2.VideoCapture('http://192.168.2.2:8081/')  # 웹캠 URL을 설정
    while True:
        ret, latest_frame = cap.read()
        if not ret:
            break
        frame = latest_frame  # 최신 프레임 저장
    cap.release()

def find_green(img, size = 10):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    green_lower = np.array([70, 10, 230], np.uint8)
    green_upper = np.array([95, 120, 255], np.uint8)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > size * size:
            return "green"
    return None

def find_red(img, size = 10):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    red_lower1 = np.array([0, 25, 150], np.uint8)
    red_upper1 = np.array([10, 135, 255], np.uint8)
    red_lower2 = np.array([170, 130, 150], np.uint8)
    red_upper2 = np.array([180, 255, 255], np.uint8)
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > size * size:
            return "red"
    return None

# 비디오 스트리밍을 위한 쓰레드 시작
thread = threading.Thread(target=video_stream)
thread.start()

try:
    while True:
        if frame is not None:
            start_time = time.time()
            
            results = model(frame)

            message = "No objects detected"
            
            # 색상 검출
            green_message = find_green(frame, size)
            red_message = find_red(frame, size)
            
            if green_message:
                message = green_message
            elif red_message:
                message = red_message

            if results[0].boxes:
                for box in results[0].boxes:
                    if box.conf >= 0.4:
                        message = results[0].names[int(box.cls)]
                        break  # 첫 번째 유효한 객체만 메시지로 설정

            # 서버로 메시지 전송
            client_socket.send(message.encode())
            
            # 결과를 이미지에 그리기
            annotated_frame = results[0].plot()
            
            # FPS 계산
            end_time = time.time()
            fps = 1 / (end_time - start_time)
            
            # FPS 텍스트 표시
            # cv2.putText(annotated_frame, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 결과를 화면에 표시
            # cv2.imshow('YOLOv10 Real-time Detection',annotated_frame)
        
        # 'q'를 눌러 스트리밍을 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    client_socket.close()
    cv2.destroyAllWindows()
