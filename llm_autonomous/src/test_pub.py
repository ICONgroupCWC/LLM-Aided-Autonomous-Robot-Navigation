#!/usr/bin/env python3

import time
import rospy;
from std_msgs.msg import String

def main():
    corection = True
    topic = '/target_location';

    node = rospy.init_node("pub_node",anonymous=True);

    publisher = rospy.Publisher(topic, String,queue_size=10);
    
    while not rospy.is_shutdown():

        message = 'coffee room';

        if(corection==True):
            #for i in range(1000000):
            time.sleep(3)
            publisher.publish(message);
            print("Successfully published target location")
            corection = False

        

if __name__ == '__main__':
    try:
        main();
    except:
        rospy.logerr('err in publishing');
        pass;