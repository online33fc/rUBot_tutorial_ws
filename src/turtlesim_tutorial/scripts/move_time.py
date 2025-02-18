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

def move_turtle(lin_vel,ang_vel,distance):
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
 
        if(robot_x >= distance or robot_y >= distance):
            rospy.loginfo("Robot hits a wall")
            rospy.logwarn("Stopping robot")
            break
        pub.publish(vel)
        rate.sleep()

def move_rubot(lin_velx,ang_vel,time_duration):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # Crear el publicador
    rospy.sleep(1)  # Esperar para asegurar que el publicador está listo
    rate = rospy.Rate(10)  # Frecuencia de ejecución (10 Hz)
    
    vel = Twist()  # Crear un mensaje de velocidad
    time_begin = rospy.Time.now()  # Guardar el tiempo de inicio

    while not rospy.is_shutdown():  # Ejecutar mientras ROS esté activo
        time_end = rospy.Time.now()
        duration = time_end - time_begin  # Calcular tiempo transcurrido
        duration_s = duration.to_sec()  # Convertir a segundos

        if duration_s <= time_duration:
            rospy.loginfo("Robot running")
            vel.linear.x = lin_velx  # Asignar velocidad lineal
            vel.angular.z = ang_vel  # Asignar velocidad angular
        else:
            rospy.logwarn("Stopping robot")
            vel.linear.x = 0  # Detener el robot
            vel.angular.z = 0

        pub.publish(vel)  # Publicar la velocidad
        rate.sleep()  # Mantener la tasa de ejecución

if __name__ == '__main__':
    try:
        rospy.init_node('move_turtle', anonymous=False)#Has to be called here at the begining!
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        t = rospy.get_param("~t")
        move_rubot(v, w, t)  # Cridem a la funcion nova
    except rospy.ROSInterruptException:
        pass