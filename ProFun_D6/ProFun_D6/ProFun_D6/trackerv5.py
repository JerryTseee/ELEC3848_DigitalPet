################################################################################
## Jetson Nano
## Data is sent to Arduino Mega through serial communication.
## Multithreading to reduce delay.
################################################################################

import jetson_inference
import jetson_utils
import cv2
import numpy as np
import mediapipe as mp
import serial
import time
import json
from datetime import datetime
from multiprocessing import Process, Queue
import queue

WIDTH = 320
HEIGHT = 240

def frame_capture(q, max_fps=10):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    assert cap.isOpened(), "Failed to open camera."
    interval = 1.0 / max_fps

    while True:
        start = time.time()
        ret, frame = cap.read()
        if ret:
            if not q.full():
                q.put_nowait(frame)
        elapsed = time.time() - start
        time.sleep(max(0, interval - elapsed))
        
def inference_worker(frame_q):
    
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0) # Non-blocking
    time.sleep(2)

    # Load the object detection network
    net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
    
    # For hand detection
    mp_Hands = mp.solutions.hands
    hands = mp_Hands.Hands(
        static_image_mode=False,
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        max_num_hands=1)

    mpDraw = mp.solutions.drawing_utils
    finger_Coord = [(8, 6), (12, 10), (16, 14), (20, 18)]
    thumb_Coord = (4, 3)

    # Image dimensions
    coordinates = np.array([WIDTH//2, HEIGHT//2])
    width, height = 0, 0
    prev_count = -1
    curr_count = -1
    follow = False
    prev_gesture = False
    timer = 0
    packetno = 1
    
    while True:
        try:
            while frame_q.qsize() > 1:
                frame_q.get_nowait()  # Drop old frames

            frame = frame_q.get(timeout=0)

            # Preprocess            
            img = cv2.resize(frame, (WIDTH, HEIGHT))
            img_ = img.copy()
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # === Hand Detection ===
            result = hands.process(img_rgb)

            prev_count = curr_count
            curr_count = -1  # Default to -1
            if result.multi_hand_landmarks:
                curr_count = 0
                for _, handLms in enumerate(result.multi_hand_landmarks):
                    mpDraw.draw_landmarks(img_, handLms, mp_Hands.HAND_CONNECTIONS)
                    handList = [(int(lm.x * WIDTH), int(lm.y * HEIGHT)) for lm in handLms.landmark]

                    for coord in finger_Coord:
                        if handList[coord[0]][1] < handList[coord[1]][1]:
                            curr_count += 1

                    side_thumb = "left" if handList[17][0] < handList[5][0] else "right"
                    if side_thumb == "left":
                        if handList[thumb_Coord[0]][0] < handList[thumb_Coord[1]][0]:
                            curr_count += 1
                    else:
                        if handList[thumb_Coord[0]][0] > handList[thumb_Coord[1]][0]:
                            curr_count += 1

            # === Gesture Toggle Follow Mode ===
            if curr_count == 5 and not prev_gesture:
                follow = not follow
            prev_gesture = (curr_count == 5)
            # === Human Detection ===
            if follow:
                img_rgba = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2RGBA).astype(np.float32)
                cuda_img = jetson_utils.cudaFromNumpy(img_rgba)

                detections = net.Detect(cuda_img, WIDTH, HEIGHT)
                filtered_detections = [d for d in detections if d.ClassID == 1 and d.Confidence > 0.7]

                if filtered_detections:
                    coordinates_ = []
                    widths = []
                    heights = []
                    for detection in filtered_detections:
                        cx, cy = detection.Center
                        coordinates_.append([int(cx), int(cy)])
                        widths.append(int(detection.Width))
                        heights.append(int(detection.Height))

                    coordinates_ = np.array(coordinates_)
                    diffs = coordinates_ - coordinates
                    distances = np.linalg.norm(diffs, axis=1)
                    min_idx = np.argmin(distances)
                    coordinates = coordinates_[min_idx]
                    width = widths[min_idx]
                    height = heights[min_idx]

                    cv2.circle(img_, tuple(coordinates), 5, (255, 0, 0), -1)
                else:
                    coordinates = np.array([0, 0])
                    width, height = 0, 0
            else:
                coordinates = np.array([0, 0])
                width, height = 0, 0

            # === Serial Communication ===
            cx, cy = coordinates
            packet = {
                "curr_count": curr_count,
                "follow_mode": follow,
                "cx": int(cx),
                "cy": int(cy),
                "width": width,
                "height": height,
            }

            json_packet = json.dumps(packet) + '\n'
            print(json_packet)
            # print(packetno, datetime.now().strftime("%H:%M:%S"))
            packetno += 1

            if ser.is_open:
                ser.write(json_packet.encode('utf-8'))

            # # === Display ===
            # cv2.putText(img_, str(curr_count), (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (0, 220, 100), 2)
            # cv2.imshow('Tracking', img_)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except queue.Empty:
            continue

def run():

    frame_q = Queue(maxsize=2)

    capture_p = Process(target=frame_capture, args=(frame_q,))
    inference_p = Process(target=inference_worker, args=(frame_q,))
    
    capture_p.start()
    inference_p.start()

    capture_p.join()
    inference_p.join()

run()
