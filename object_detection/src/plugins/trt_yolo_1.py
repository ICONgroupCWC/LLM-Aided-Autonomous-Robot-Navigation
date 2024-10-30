#!/usr/bin/env python3

import rospy
import os
import time
import argparse
import cv2
import pycuda.autoinit 
import ctypes
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
from trt_model import detect
from trt_model import get_input_shape
from trt_model import allocate_buffers
import pyrealsense2 as rs 
from sensor_msgs.msg import CompressedImage
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String

goal_flag = True
obj_id = None

def callback1(msg):
    global goal_flag
    if(msg.status_list !=[]):
       
        if(msg.status_list[0].status ==3):
            goal_flag = False

        else:
            goal_flag = True

def callback2(msg):
    global obj_id
    obj_id = msg.data

def load_engine(model):
    try:
        ctypes.cdll.LoadLibrary('/home/jetbot/catkin_ws/src/LLM_project/object_detection/src/plugins/libyolo_layer.so')
    except OSError as e:
        raise SystemExit('ERROR: failed to load ./plugins/libyolo_layer.so.  '
                     'Did you forget to do a "make" in the "./plugins/" '
                     'subdirectory?') from e


    trt_logger = trt.Logger(trt.Logger.INFO)
    TRTbin = model 
    with open(TRTbin, 'rb') as f, trt.Runtime(trt_logger) as runtime:
        return runtime.deserialize_cuda_engine(f.read())


def draw_boxes_cv2(img, boxes, confs, clss,cls_dict): 

    
    red = (0, 0, 255)       
    blue = (255, 0, 0)      
    green = (0, 255, 0)     
    indigo = (75, 0, 130)   
    orange = (0, 140, 255)  

    colors = [red, blue, green, indigo, orange]
    for i in range(len(boxes)):
        
        class_name = cls_dict[clss[i]]
        x1 = int(boxes[i][0])
        y1 = int(boxes[i][1])
        x2 = int(boxes[i][2])
        y2 = int(boxes[i][3])
        img = cv2.rectangle(img, (x1, y1), (x2, y2), colors[int(clss[i])], 2)

        if((y1-25)>0):
            prx1 = x1
            pry1 = y1-4

            rx1 = x1
            ry1 = y1-23
            rx2 = x1+195
            ry2 = y1

        else:
            prx1 = x1
            pry1 = y2+15

            rx1 = x1
            ry1 = y2
            rx2 = x1+195
            ry2 = y1+23

        string1 = class_name + "  " + str(round(confs[i], 2))
        img = cv2.rectangle(img, (rx1, ry1), (rx2, ry2), colors[int(clss[i])], -1)
        img = cv2.putText(img, string1, (prx1, pry1), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255,255,255), 1)

    
    return img

def constrain_to_range(x):
    if x > 0.7:
        return 0.7
    elif x < -0.7:
        return -0.7
    else:
        return x

def main():
    global goal_flag
    global obj_id

    move_cmd = Twist()

    topic1 = '/img_'
    topic2 = '/move_base/status'
    topic3 = '/cmd_vel'
    topic4 = '/object_id'

    rospy.init_node('object_detection',anonymous=True);

    pub1 = rospy.Publisher(topic1, CompressedImage, queue_size=10)
    pub2 = rospy.Publisher(topic3, Twist,queue_size=10);

    rospy.Subscriber(topic2, GoalStatusArray, callback1)
    rospy.Subscriber(topic4, String, callback2)

    pipe = rs.pipeline()
    cfg = rs.config()

    cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)

    pipe.start(cfg)


    model = '/home/jetbot/catkin_ws/src/LLM_project/object_detection/src/yolov4_model.trt'
    letter_box = True
    conf_th = 0.50
    cuda_ctx = None
 
    cls_dict = {0:'bin',1:'coffee machine' ,2:'professor' ,3:'robot' ,4:'tv'}
    
    
    engine = load_engine(model)
    input_shape = get_input_shape(engine)
    

    # Set host input to the image. The do_inference() function
    # will copy the input to the GPU before executing.
    if cuda_ctx:
        cuda_ctx.push()

    try:
        context = engine.create_execution_context()
        inputs, outputs, bindings, stream = allocate_buffers(engine)
                
    except Exception as e:
        raise RuntimeError('fail to allocate CUDA resources') from e
    finally:
        if cuda_ctx:
            cuda_ctx.pop()
    
    loop_ = 0
    kp = 0.01
    kd = 0.1
    prv_err = 0
    loop_exit = False
    while not rospy.is_shutdown():
        
        print(obj_id)
        frame = pipe.wait_for_frames()

        depth_frame = frame.get_depth_frame()
        color_frame = frame.get_color_frame()

        image_src = np.asanyarray(color_frame.get_data())
        image_src = cv2.resize(image_src, (720, 405))
        
        if(goal_flag==False):
            boxes, confs, clss = detect(engine,image_src,input_shape,inputs, outputs, bindings,context, stream,conf_th)
            image_src = draw_boxes_cv2(image_src, boxes, confs, clss,cls_dict)

            if(len(confs)>0):
                print('detected')
                
                conf_max = confs.max(axis=0)
                mx_indx = np.argmax(confs)
                cls_id = cls_dict[clss[mx_indx]]                
                
                if(obj_id is not None and obj_id==cls_id):

                    mid_x = int((int(boxes[mx_indx][0])+ int(boxes[mx_indx][2]))/2)
                    mid_y = int((int(boxes[mx_indx][1])+ int(boxes[mx_indx][3]))/2)
                    error = (image_src.shape[1]/2)-mid_x
                    
                    rot = kp*error + kd*(error-prv_err)
                
                    rot = constrain_to_range(rot)
                    
                    cv2.line(image_src, (mid_x,mid_y), (int(image_src.shape[1]/2),mid_y), (0,255,0), 2)
                    prv_err = error
                    if(abs(error)<20):
                        move_cmd.linear.x = 0.0
                        move_cmd.linear.y = 0.0
                        move_cmd.linear.z = 0.0
                        
                        move_cmd.angular.x = 0.0
                        move_cmd.angular.y = 0.0
                        move_cmd.angular.z = 0.0
                    else:
                        move_cmd.linear.x = 0.0
                        move_cmd.linear.y = 0.0
                        move_cmd.linear.z = 0.0
                        
                        move_cmd.angular.x = 0.0
                        move_cmd.angular.y = 0.0
                        move_cmd.angular.z = rot
                
                elif(obj_id is None ):
                    move_cmd.linear.x = 0.0
                    move_cmd.linear.y = 0.0
                    move_cmd.linear.z = 0.0
                    
                    move_cmd.angular.x = 0.0
                    move_cmd.angular.y = 0.0
                    move_cmd.angular.z = 0.0
                    print('object is None')

                elif(obj_id !=cls_id):

                    for i in range(len(clss)):
                        if(cls_dict[clss[i]]==obj_id):
                            print('object is detected')
                            mid_x = int((int(boxes[i][0])+ int(boxes[i][2]))/2)
                            mid_y = int((int(boxes[i][1])+ int(boxes[i][3]))/2)
                            error = (image_src.shape[1]/2)-mid_x
                    
                            rot = kp*error + kd*(error-prv_err)
                
                            rot = constrain_to_range(rot)
                    
                            cv2.line(image_src, (mid_x,mid_y), (int(image_src.shape[1]/2),mid_y), (0,255,0), 2)
                            prv_err = error

                            if(abs(error)<20):
                                move_cmd.linear.x = 0.0
                                move_cmd.linear.y = 0.0
                                move_cmd.linear.z = 0.0
                                
                                move_cmd.angular.x = 0.0
                                move_cmd.angular.y = 0.0
                                move_cmd.angular.z = 0.0
                            else:
                                move_cmd.linear.x = 0.0
                                move_cmd.linear.y = 0.0
                                move_cmd.linear.z = 0.0
                                
                                move_cmd.angular.x = 0.0
                                move_cmd.angular.y = 0.0
                                move_cmd.angular.z = rot
                        else:
                            print('can not find the object')
                            move_cmd.linear.x = 0.0
                            move_cmd.linear.y = 0.0
                            move_cmd.linear.z = 0.0
                            
                            move_cmd.angular.x = 0.0
                            move_cmd.angular.y = 0.0
                            move_cmd.angular.z = 0.0

                    
                    
            else:
                
                print('not detected')

                move_cmd.linear.x = 0.0
                move_cmd.linear.y = 0.0
                move_cmd.linear.z = 0
                
                move_cmd.angular.x = 0.0
                move_cmd.angular.y = 0.0
                move_cmd.angular.z = 1.0
                

            pub2.publish(move_cmd)
            
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_src)[1]).tostring()


        pub1.publish(msg)
        #cv2.imshow('result', image_src) 
        
        #cv2.waitKey(1)


if __name__ == "__main__":
    main();
    # try:
    #     main();
    # except:
    #     rospy.logerr('Subscriber is error');
    #     pass;
