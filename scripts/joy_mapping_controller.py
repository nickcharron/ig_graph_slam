#!/usr/bin/env python
import rospy
import rospkg
import os
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Initialize variables
buttons_array = [0, 0]
collect_bag_btn_num = 4
collect_bag_btn_sym = "L1"
stop_bag_btn_num = 5
stop_bag_btn_sym = "R1"

def joy_CB(joy_msg):
    global buttons_array
    buttons_array = [joy_msg.buttons[collect_bag_btn_num],joy_msg.buttons[stop_bag_btn_num]]

def print_instructions():

    print ""
    print "Press %s to start collecting a loam bag" % collect_bag_btn_sym
    print "Press %s to stop collecting loam bag and run graph slam" % stop_bag_btn_sym
    print ""

def launch_subscribers():
    rospy.init_node('joy_mapping_control')
    rospy.Subscriber("/joy_teleop/joy", Joy, joy_CB )

def check_buttons():

    # Start collecting bag
    if buttons_array[0] == 1:
        while buttons_array[0] == 1:    # Wait for button to be released
            pass
        print "START button selected."
        os.system('rosrun ig_graph_slam collect_loam_gs_bag.sh &')

    if buttons_array[1] == 1:
        while buttons_array[1] == 1:    # Wait for button to be released
            pass
        print "STOP button selected."
        os.system('rosrun ig_graph_slam start_graph_slam.sh')

def main():
    # start node to subscribe to joy messages node end messages
    launch_subscribers()

    # check buttons and launch the appropriate file
    while not rospy.is_shutdown():
        check_buttons()
    rospy.spin()

if __name__ == '__main__':

    print_instructions()

    main()
