#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

robot_x = 0
robot_y = 0

def pose_callback(pose):
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot X = %f\t Robot Y = %f\n",pose.x, pose.y)

def move_turtle(lin_vel,ang_vel,distance, min_x, max_x, min_y,max_y):
    global robot_x, robot_y
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose',Pose, pose_callback)
    rate = rospy.Rate(10) # 10hz
    vel = Twist()
    while not rospy.is_shutdown():
        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel
 
        if(robot_x < min_x or robot_x > max_x or robot_y < min_y or robot_y > max_y):
            rospy.loginfo("Robot hits a wall")
            rospy.logwarn("Stopping robot")
            break
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('move_turtle', anonymous=False)#Has to be called here at the begining!
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        d= rospy.get_param("~d")
        min_x = rospy.get_param("~min_x")  
        max_x = rospy.get_param("~max_x")
        min_y = rospy.get_param("~min_y")
        max_y = rospy.get_param("~max_y")

        # Llamada a la función con los parámetros correctos
        move_turtle(v, w, d, min_x, max_x, min_y, max_y)
    except rospy.ROSInterruptException:
        pass