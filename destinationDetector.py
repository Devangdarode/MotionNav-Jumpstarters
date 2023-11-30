import sys
import time

import ecal.core.core as ecal_core
from RecognizedGesture_pb2 import Gesture
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.subscriber import StringSubscriber


# Callback for receiving messages
def callbackGesture(topic_name, msg, time):
    destination = ""
    # print("Received: {}".format(msg))
    if "Closed_Fist" in str(msg):
        print ("Going Home")
        destination = "Going Home"
    elif "Open_Palm" in str(msg):
        print ("Going Office")
        destination = "Going Office"
    elif "Thumb_Down" in str(msg):
        print ("Going to the Bar")
        destination = "Going to the Bar"
    elif "Thumb_Up" in str(msg):
        print ("Going the Gym")
        destination = "Going the Gym"
    return destination

def callbackUser(topic_name, msg, time):
    user = "no user"
    user == str(msg)
    return user

if __name__ == "__main__":
    # Initialize eCAL and give a process name
    ecal_core.initialize(sys.argv, "HandGesture to destinated detector")

    # Creating a subscriber that listens to "name/Face" topic
    subUser = StringSubscriber("Face/name")

    # Creating a subscriber that listens to "gestures/Webcam" topic
    subGesture = ProtoSubscriber("gestures/Webcam", Gesture)

    # Set the Callback
    destination = subGesture.set_callback(callbackGesture)
    print ("The destination callback is : {}".format(destination))
    
    userDetected = subUser.set_callback(callbackUser)
    print ("The User callback is : {}".format(userDetected))
  
    # Just don't exit
    while ecal_core.ok():
        time.sleep(0.5)
    
    # finalize eCAL API
    ecal_core.finalize()