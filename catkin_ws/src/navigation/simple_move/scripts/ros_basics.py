#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

NAME = "Luis Yehosu"

def callback_scan(msg):
    global obstacle_detected
    n = int((msg.angle_max - msg.angle_min)/msg.angle_increment/2)
    obstacle_detected = msg.ranges[n] < 1.0

def arm_move(msg):
    rospy.loginfo("Datos Recibidos: %s", msg.data)

    return

def main():
    print("ROS BASICS - " + NAME)
    rospy.init_node("ros_basics")
    
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rospy.Subscriber("/hardware/left_arm/goal_pose", Float64MultiArray, arm_move)
    pub_larm_pose = rospy.Publisher("/hardware/left_arm/goal_pose", Float64MultiArr, queue_size=10)

    #pub_la1 = rospy.Publisher("/la_1_controller_command", float64,queue_size=10)
    loop = rospy.Rate(10)

    global obstacle_detected
    obstacle_detected = False

    while not rospy.is_shutdown():
        msg_cmd_vel = Twist()
        msg_la_pose = Float64MultiArray()

        msg_la_pose.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub_larm_pose.publish(msg_la_pose)
        msg_cmd_vel.linear.x = 0 if obstacle_detected else 0.3
        pub_cmd_vel.publishing(msg_cmd_vel)
        if msg_cmd_vel.linear.x == 0:
            msg_la_pose.data = [1.0416, -0.0001, 0.5003, -0.5000, 1.0999, -0.2880, 0.0001]
            pub_larm_pose.publish(msg_la_pose)
        loop.sleep()

if _name_ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
