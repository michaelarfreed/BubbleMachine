# TechVidvan hand Gesture Recognizer

# import necessary packages

import cv2
import json
import numpy as np
import mediapipe as mp
import tensorflow as tf
from tensorflow.keras.models import load_model
import requests
from PIL import Image
from io import BytesIO

# initialize mediapipe
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mpDraw = mp.solutions.drawing_utils

# Load the gesture recognizer model
model = load_model('mp_hand_gesture')

# Load class names
f = open('gesture.names', 'r')
classNames = f.read().split('\n')
f.close()
print(classNames)

camera_ip = "http://192.168.4.2"
esp_ip = "http://192.168.4.1"

get_image_link = camera_ip + "/capture"
send_result_link = esp_ip + "/get_gesture"

# for smoothing
prev_className = ""
n_className = 0


while True:
    response = requests.get(get_image_link)
    image_stream = BytesIO(response.content)
    frame = cv2.imdecode(np.frombuffer(image_stream.read(), np.uint8), 1)
    x, y, c = frame.shape

    # Flip the frame vertically
    frame = cv2.flip(frame, 1)
    framergb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Get hand landmark prediction
    result = hands.process(framergb)

    # print(result)

    className = ''

    # post process the result
    if result.multi_hand_landmarks:
        landmarks = []
        for handslms in result.multi_hand_landmarks:
            for lm in handslms.landmark:
                # print(id, lm)
                lmx = int(lm.x * x)
                lmy = int(lm.y * y)

                landmarks.append([lmx, lmy])

            # Drawing landmarks on frames
            mpDraw.draw_landmarks(frame, handslms, mpHands.HAND_CONNECTIONS)

            # Predict gesture
            prediction = model.predict([landmarks])
            # print(prediction)
            classID = np.argmax(prediction)
            prev_className = className
            className = classNames[classID]
            if prev_className == className:
                n_className += 1
            else:
                n_className = 0

    # show the prediction on the frame
    cv2.putText(frame, className, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (0,0,255), 2, cv2.LINE_AA)

    # Show the final output
    cv2.imshow("Output", frame) 

    # some smoothing. send the gesture if you've seen it 6 times
    if n_className > 6 or className == "": 
        payload = dict(hand_gesture=className)
        requests.post(send_result_link, json=payload)

    if cv2.waitKey(1) == ord('q'):
        break

# release the webcam and destroy all active windows
# cap.release()

cv2.destroyAllWindows()
