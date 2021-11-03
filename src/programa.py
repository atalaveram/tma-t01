#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
def command_callback(msg):
    comando=raw_input("Ingrese comando: ")
    if comando[:7]=="Avanza ":
        velocidad.linear.x=float(comando[7:])
    elif comando[:5]=="Gira ":
        velocidad.angular.z=float(comando[5:])
    elif comando=="Detente":
        velocidad.linear.x=0
        velocidad.angular.z=0
    else:
        rospy.loginfo("Error")
    pub.publish(velocidad)
rospy.init_node('robot_comm')
sub=rospy.Subscriber('odom',Odometry,command_callback)
pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
rate=rospy.Rate(2)
velocidad=Twist()
rospy.spin()