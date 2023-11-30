import cv2
import mediapipe as mp
import time
import numpy as np

# Grabbing the Holistic Model from Mediapipe and
# Initializing the Model
mp_holistic = mp.solutions.holistic
holistic_model = mp_holistic.Holistic(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# Initializing the drawing utils for drawing the facial landmarks on image
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# (0) in VideoCapture is used to connect to your computer's default camera
cap = cv2.VideoCapture(0)
hands = mp_hands.Hands(max_num_hands =1, min_detection_confidence=0.5,min_tracking_confidence=0.5) 

def main():
    with hands:
        while cap.isOpened():
            success, image = cap.read()

            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            results = hands.process(image)

            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    #         image_height, image_width, _ = image.shape
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    p4_x = hand_landmarks.landmark[mp_hands.HandLandmark(4).value].x
                    p4_y = hand_landmarks.landmark[mp_hands.HandLandmark(4).value].y
                    p6_x = hand_landmarks.landmark[mp_hands.HandLandmark(6).value].x
                    p6_y = hand_landmarks.landmark[mp_hands.HandLandmark(6).value].y
                    p12_x = hand_landmarks.landmark[mp_hands.HandLandmark(12).value].x
                    p12_y = hand_landmarks.landmark[mp_hands.HandLandmark(12).value].y
                    p0_x = hand_landmarks.landmark[mp_hands.HandLandmark(0).value].x
                    p0_y = hand_landmarks.landmark[mp_hands.HandLandmark(0).value].y
                    p1_x = hand_landmarks.landmark[mp_hands.HandLandmark(1).value].x
                    p1_y = hand_landmarks.landmark[mp_hands.HandLandmark(1).value].y
                    
                    mp_drawing.draw_landmarks(
                        image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    scalingFactor = distance(p0_x, p0_y, p1_x, p1_y)
                    throttle = 1 - (distance(p4_x, p4_y, p6_x, p6_y)/(2*scalingFactor))
                    if throttle < 0.2:
                        throttle = 0
                    elif throttle > 1:
                        throttle = 1
                    throttle = round(throttle,4)
                    brake = distance (p12_x, p12_y, p0_x, p0_y)/(5*scalingFactor)
                    if brake < 0.5:
                        brake = 0
                    elif brake > 1:
                        brake = 1
                        throttle = 0
                    else:
                        throttle = 0
                    brake = round(brake,4)
                    steer = steerAngle(angle(p0_x, p0_y, p6_x, p6_y))
                    print(str(throttle) + "  " + str(brake) + "  " + str(steer))
            cv2.imshow('MediaPipe Hands', image)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
    cap.release()
    cv2.destroyAllWindows()

def distance(x1, y1, x2, y2):
    distance = (((x2-x1)**2)+((y2-y1)**2))**0.5
    return distance

# read the angle between 80 and 120 degree 
def angle(x1, y1, x2, y2):
    dx = x2-x1
    dy = y2-y1
    rawAngle = (np.arctan(dy/dx)*180/np.pi) # Negative in second quadrant
    if rawAngle < 0:
        angle = 180 +rawAngle
    else:
        angle = rawAngle
    if angle < 80:
        angle = 80
    elif angle > 120:
        angle = 120
    return angle

# map the raw angle (40 degree range) to steer value [-1.0, 1.0]
# positive steer angle is to the right
def steerAngle(angle):
    delta = 100 - angle
    
    if (angle < 103 and angle > 97):
        steeringAngle = 0
    elif (angle >= 103 and angle <= 97):
        steeringAngle = (delta)/20
    else:
       pass

    steeringAngle = (delta)/20
    return steeringAngle


if __name__ == "__main__":
    main()