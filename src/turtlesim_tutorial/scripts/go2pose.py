#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self):
        rospy.init_node('move_turtle', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.goal_pose = Pose()
        self.goal_pose.x = rospy.get_param("~x")
        self.goal_pose.y = rospy.get_param("~y")
        self.goal_pose.theta = rospy.get_param("~theta", 0.0)  # Se añade orientación deseada
        self.distance_tolerance = rospy.get_param("~tol")
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback para actualizar la pose actual de la tortuga"""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)

    def euclidean_distance(self, goal_pose):
        """Distancia euclidiana entre la posición actual y la meta"""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """Velocidad lineal proporcional a la distancia"""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """Ángulo de dirección deseado hacia la meta"""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, angle, constant=6):
        """Velocidad angular proporcional al error de orientación"""
        return constant * (angle - self.pose.theta)

    def move2goal(self):
        """Primero mueve la tortuga a la posición deseada, luego ajusta la orientación"""
        goal_pose = Pose()
        goal_pose.x = self.goal_pose.x
        goal_pose.y = self.goal_pose.y

        distance_tolerance = self.distance_tolerance
        vel_msg = Twist()

        # FASE 1: Moverse hacia la meta sin ajustar la orientación final
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(self.steering_angle(goal_pose))
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Detener el movimiento lineal al llegar a la posición
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.loginfo("Posición alcanzada, ajustando orientación...")

        # FASE 2: Ajustar la orientación final
        while abs(self.goal_pose.theta - self.pose.theta) >= 0.01:
            vel_msg.angular.z = self.angular_vel(self.goal_pose.theta, constant=2.0)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Detener completamente
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.loginfo("Orientación ajustada, destino alcanzado")

        rospy.spin()


if __name__ == '__main__':
    try:
        turtle = TurtleBot()
        turtle.move2goal()
    except rospy.ROSInterruptException:
        pass
