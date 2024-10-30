#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


check_location = False;
traget_location = None;
target_goal = PoseStamped()

def callback1(message: String):
    global check_location,traget_location;
    print(message.data)
    traget_location = message.data

    check_location = True
    


def read_file(path, location):
    pos_x = 0
    pos_y = 0
    pos_z = 0
    or_x = 0
    or_y = 0
    or_z = 0
    or_w = 0
    with open(path, 'r') as file:
        for line in file:
            elements = line.strip().split(",")
            if(elements[0]==location):
                pos_x = float(elements[1])
                pos_y = float(elements[2])
                pos_z = float(elements[3])
                or_x = float(elements[4])
                or_y = float(elements[5])
                or_z = float(elements[6])
                or_w = float(elements[7])
                
    return  pos_x,pos_y,pos_z,or_x,or_y,or_z,or_w   
        



def main():
    global check_location, traget_location,current_goal;
    rospy.init_node('target_pos_pub',anonymous=True);

    topic1 = '/target_location';
    topic2 = '/move_base_simple/goal';
    #topic3 = '/odom'


    file_path = '/home/jetbot/catkin_ws/src/LLM_project/llm_autonomous/src/location_data.txt'
    rospy.Subscriber(topic1,String,callback1)

    pub1 = rospy.Publisher(topic2, PoseStamped,queue_size=10);

    while not rospy.is_shutdown():
        
        if(check_location==True and traget_location is not None):
            pos_x,pos_y,pos_z,or_x,or_y,or_z,or_w = read_file(file_path, traget_location)

            target_goal.header.frame_id = "map"
            target_goal.pose.position.x = pos_x
            target_goal.pose.position.y = pos_y
            target_goal.pose.position.z = pos_z
            target_goal.pose.orientation.x = or_x
            target_goal.pose.orientation.y = or_y
            target_goal.pose.orientation.z = or_z
            target_goal.pose.orientation.w = or_w

            time.sleep(3)
            pub1.publish(target_goal)
            print("Successfully published target location"+"("+traget_location+")")
            check_location=False

if __name__ == '__main__':

    try:
        main();
    except:
        rospy.logerr('Subscriber is error');
        pass;