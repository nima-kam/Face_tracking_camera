import cv2
# import numpy as np
# import mediapipe as mp
import requests
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import paho.mqtt.client as mqtt
import paho.mqtt.publish as pub
import ssl

# MQTT broker details
broker_address = "neee5c55.ala.us-east-1.emqxsl.com"
broker_port = 8883

client_id='python'
username = "pythonclient"
password = "pythonclient"
print('st mqtt client')
# MQTT topic and message
topic = "esp32/servo/change"
message = "Hello, MQTT!"


# pub.single(topic="script/hello", payload=message,hostname=broker_address,port=broker_port,client_id=client_id,auth={'username':username, 'password':password})
# Define callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

def on_publish(client, userdata, mid):
    print("Message published with id " + str(mid)+" the data is:",userdata)
def on_connect_failure(client, userdata, flags, rc):
    print("Connection to MQTT broker failed with result code " + str(rc))

# Create MQTT client
client = mqtt.Client("", True, None,protocol=mqtt.MQTTv31 )
client.on_connect=on_connect
client.on_publish=on_publish
client.on_connect_fail=on_connect_failure
# Set username and password if required
client.username_pw_set(username, password)
# client.tls_set(cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2)
client.tls_set_context()

# Connect to the broker
client.connect(broker_address, broker_port,)

print('st mqtt client complete',client.is_connected())
# Publish the message on the topic
client.publish("script/hello", message)

# Disconnect from the broker
client.disconnect()
'''
INFO SECTION
- if you want to monitor raw parameters of ESP32CAM, open the browser and go to http://192.168.x.x/status
- command can be sent through an HTTP get composed in the following way http://192.168.x.x/control?var=VARIABLE_NAME&val=VALUE (check varname and value in status)
'''

# ESP32 URL
URL = "http://192.168.1.6"
AWB = True

# Face recognition and opencv setup
cap = cv2.VideoCapture(URL + ":81/stream")
# face_classifier = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml') # insert the full path to haarcascade file if you encounter any problem
print("video got")
# STEP 2: Create an HandLandmarker object.
base_options = python.BaseOptions(model_asset_path='./hand_landmarker.task')
options = vision.HandLandmarkerOptions(base_options=base_options,
                                       num_hands=2)
detector = vision.HandLandmarker.create_from_options(options)
print("******detector set****************")
# mpHands = mp.solutions.hands
# hands = mpHands.Hands(max_num_hands=1)
# mpDraw = mp.solutions.drawing_utils


def set_resolution(url: str, index: int=1, verbose: bool=False):
    try:
        if verbose:
            resolutions = "10: UXGA(1600x1200)\n9: SXGA(1280x1024)\n8: XGA(1024x768)\n7: SVGA(800x600)\n6: VGA(640x480)\n5: CIF(400x296)\n4: QVGA(320x240)\n3: HQVGA(240x176)\n0: QQVGA(160x120)"
            print("available resolutions\n{}".format(resolutions))

        if index in [10, 9, 8, 7, 6, 5, 4, 3, 0]:
            requests.get(url + "/control?var=framesize&val={}".format(index))
        else:
            print("Wrong index")
    except:
        print("SET_RESOLUTION: something went wrong")

def set_quality(url: str, value: int=1, verbose: bool=False):
    try:
        if value >= 10 and value <=63:
            requests.get(url + "/control?var=quality&val={}".format(value))
    except:
        print("SET_QUALITY: something went wrong")

def set_awb(url: str, awb: int=1):
    try:
        awb = not awb
        requests.get(url + "/control?var=awb&val={}".format(1 if awb else 0))
    except:
        print("SET_QUALITY: something went wrong")
    return awb

videona=0
if __name__ == '__main__':
    # set_resolution(URL, index=6)
    print("*******In the Main************")
    while True:
        
        if cap.isOpened():
            videona//=2
            ret, frame = cap.read()
            # print("****cap is open******")
            if ret:
                print("***ret is okay*****",)
                # imageRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                imageRGB=frame
                # print("imagergb>",imageRGB.shape)
                face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
                # Convert the image to grayscale
                gray = cv2.cvtColor(imageRGB, cv2.COLOR_BGR2GRAY)
                key = cv2.waitKey(200)
                # Perform face detection
                faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(25, 25))
                for (x, y, w, h) in faces:
                    cv2.rectangle(imageRGB, (x, y), (x+w, y+h), (0, 255, 0), 2)
                if len(faces)>0:
                    (f_x, f_y, f_w, f_h)=faces[0]
                    c_x,c_y=(x+w//2, y+h//2)
                    i_h,i_w,_=imageRGB.shape
                    low_bound=i_h//3+1
                    high_bound=((i_h//3)*2)-1
                    right_bound=i_w//3+1
                    left_bound=((i_w//3)*2)-1

                    flag_move=False
                    vertical_move=0
                    if f_x<=low_bound:
                        vertical_move=1
                        flag_move=True
                    elif f_x>=high_bound:
                        vertical_move=-1
                        flag_move=True
                    
                    h_move=0
                    if f_y<=right_bound:
                        h_move=1
                        flag_move=True
                    elif f_y>=left_bound:
                        h_move=-1
                        flag_move=True
                    
                    if flag_move:
                    
                        message=f'{vertical_move},{h_move}'
                        print(f"face at point:({c_x},{c_y}), move is vertical:{vertical_move}, horizantal:{h_move}")
                        

                        # publish the message

                        # Connect to the broker
                        client.connect(broker_address, broker_port)

                        # Publish the message on the topic
                        client.publish(topic, message)

                        # Disconnect from the broker
                        client.disconnect()
                

                print("image shape:",imageRGB.shape,'  ,faces:',faces)                
            
            if ret:
                print("frame is :",frame.shape)
                cv2.imshow("frame", frame)

            else:
                key = cv2.waitKey(450)
                print("after a while")
                break
        else:
            videona+=1
            print("video not available",videona)
            key = cv2.waitKey(500)
            if videona>12:
                break

        # if key == ord('x'):
        #     break
            # if key == ord('r'):
            #     idx = int(input("Select resolution index: "))
            #     set_resolution(URL, index=idx, verbose=True)

            # elif key == ord('q'):
            #     val = int(input("Set quality (10 - 63): "))
            #     set_quality(URL, value=val)

            # elif key == ord('a'):
            #     AWB = set_awb(URL, AWB)

            # elif key == 27:
            #     break

    cv2.destroyAllWindows()
    cap.release()