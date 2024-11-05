#!/usr/bin/env python3
import cv2
from django.http import StreamingHttpResponse
from django.views.generic.base import View
from threading import Thread
from django.shortcuts import render
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from sensor_msgs.msg import Image

# img_np = np.zeros((405,720,3), np.uint8)
img_np = cv2.imread("/home/josh/chat_bot/chat_project/video_streaming/1.jpg")
img_np = cv2.resize(img_np,(720,405), interpolation=cv2.INTER_LINEAR)

def index(request):
    # Render the chat_app/index.html template
    return render(request, 'chat_app/index.html')


def callback1(message):
    
    global img_np
    np_arr = np.fromstring(message.data,np.uint8)
    img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    img_np = cv2.resize(img_np,(720,405), interpolation=cv2.INTER_LINEAR)
    print(img_np)

def gen():
    global img_np
    
    topic1 = '/img_';
    


    rospy.Subscriber(topic1,CompressedImage, callback1);
    
    while True:
       
        image = cv2.resize(img_np,(720,405))
        
        #frame_flip = cv2.flip(image, 1)
        ret, jpeg = cv2.imencode('.jpg', image)
        if not ret:
            print("Error: Unable to encode frame as JPEG")
            continue
        
        frame = jpeg.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        

def video_feed(request):
    return StreamingHttpResponse(gen(), content_type='multipart/x-mixed-replace; boundary=frame')
