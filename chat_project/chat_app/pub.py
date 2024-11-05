#!/usr/bin/env python3

import rospy
import argparse
import json

from std_msgs.msg import String




def Pub_data(target_loc,act,objec):

    topic1 = '/target_location'
    topic2 = '/object_id'
    topic3 = '/action'

    
    pub1 = rospy.Publisher(topic1, String, queue_size=10)
    pub2 = rospy.Publisher(topic2, String, queue_size=10)
    pub3 = rospy.Publisher(topic3, String, queue_size=10)
    

    if(target_loc is not None):

        print("---------------------")
        print(target_loc)
        print(objec)
        
        for i in range(10):
            pub1.publish(target_loc)
            pub2.publish(objec)
            pub3.publish(act)

    else:
        print("not available")
    

if __name__ == "__main__":

    Pub_data()