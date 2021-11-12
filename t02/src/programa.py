#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point,Quaternion,Twist
from tf.transformations import euler_from_quaternion
from math import atan2
class RobotMover(object):
    def __init__(self):
        self._distance_to_point=Float64()
        self._distance_to_point.data=0.0
        self._current_position=Point()
        self._current_orientation=Quaternion()
        self._destination=Point()
        self._destination.x=float(input('Ingrese coordenada en x: '))
        self._destination.y=float(input('Ingrese coordenada en y: '))
        self._arrived_to_destination=False
        self._velocity=Twist()
        self._angle_sections={}
        self.get_init_position_and_orientation()
        self.robot_moved_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.obstacle_avoid_sub=rospy.Subscriber('/scan',LaserScan,self.obstacle_avoid_callback)
        self.robot_moved_sub=rospy.Subscriber('/odom',Odometry,self.odom_callback)
    def get_init_position_and_orientation(self):
        data_odom=None
        while data_odom is None:
            try:
                data_odom=rospy.wait_for_message('/odom',Odometry,timeout=1)
            except Exception as e:
                rospy.logerr(e)
        data_scan=None
        while data_scan is None:
            try:
                data_scan=rospy.wait_for_message('/scan',LaserScan,timeout=1)
            except Exception as e:
                rospy.logerr(e)
        self._current_position.x=data_odom.pose.pose.position.x
        self._current_position.y=data_odom.pose.pose.position.y
        self._current_position.z=data_odom.pose.pose.position.z
        self._current_orientation.x=data_odom.pose.pose.orientation.x
        self._current_orientation.y=data_odom.pose.pose.orientation.y
        self._current_orientation.z=data_odom.pose.pose.orientation.z
        self._current_orientation.w=data_odom.pose.pose.orientation.w
        self._angle_sections={
        'right':  min(min(data_scan.ranges[270:306]),10),
        'fright': min(min(data_scan.ranges[306:342]),10),
        'front':  min(min(min(data_scan.ranges[0:18]),min(data_scan.ranges[342:359])),10),
        'fleft':  min(min(data_scan.ranges[19:54]),10),
        'left':   min(min(data_scan.ranges[55:90]),10),
        'back':   min(min(data_scan.ranges[91:269]),10)}
    def odom_callback(self,msg):
        new_position=msg.pose.pose.position
        new_orientation=msg.pose.pose.orientation
        self._distance_to_point.data=self.calculate_distance(new_position,self._destination)
        self.update_current_position(new_position)
        self.update_current_orientation(new_orientation)
        rospy.loginfo('x={:.3f} m, y={:.3f} m, distancia={:.3f} m'.format(new_position.x,new_position.y,self._distance_to_point.data))
        self.avoid_obstacles()
        self.robot_moved_pub.publish(self._velocity)
    def obstacle_avoid_callback(self,msg):
        self._angle_sections={
        'right':  min(min(msg.ranges[270:306]),10),
        'fright': min(min(msg.ranges[306:342]),10),
        'front':  min(min(min(msg.ranges[0:18]),min(msg.ranges[342:359])),10),
        'fleft':  min(min(msg.ranges[19:54]),10),
        'left':   min(min(msg.ranges[55:90]),10),
        'back':   min(min(msg.ranges[91:269]),10)}
    def update_current_position(self,new_position):
        self._current_position.x=new_position.x
        self._current_position.y=new_position.y
        self._current_position.z=new_position.z
    def update_current_orientation(self,new_orientation):
        self._current_orientation.x=new_orientation.x
        self._current_orientation.y=new_orientation.y
        self._current_orientation.z=new_orientation.z
        self._current_orientation.w=new_orientation.w
    def calculate_distance(self,new_position,old_position):
        x2=new_position.x
        x1=old_position.x
        y2=new_position.y
        y1=old_position.y
        dist=math.hypot(x2-x1,y2-y1)
        return dist
    def avoid_obstacles(self):
        if not self._arrived_to_destination:
            if self._distance_to_point.data>0.01:
                if self._angle_sections['front'] < 0.4 and self._angle_sections['fleft'] > 0.4 and self._angle_sections['fright'] > 0.4:
                    state_description = 'case 2 - front'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
                elif self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] > 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 3 - fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=1
                elif self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] < 0.4 and self._angle_sections['fright'] > 0.4:
                    state_description = 'case 4 - fleft'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
                elif self._angle_sections['front'] < 0.4 and self._angle_sections['fleft'] > 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 5 - front and fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=1
                elif self._angle_sections['front'] < 0.4 and self._angle_sections['fleft'] < 0.4 and self._angle_sections['fright'] > 0.4:
                    state_description = 'case 6 - front and fleft'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
                elif self._angle_sections['front'] < 0.4 and self._angle_sections['fleft'] < 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 7 - front and fleft and fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
                elif self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] < 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 8 - fleft and fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
                elif self._angle_sections['right'] < 0.4 and self._angle_sections['front'] > 0.4 and self._angle_sections['fright'] > 0.4:
                    state_description = 'case 9 - right'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=1
                elif self._angle_sections['left'] < 0.4 and self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] > 0.4:
                    state_description = 'case 10 - left'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
                elif self._angle_sections['right'] < 0.4 and self._angle_sections['front'] > 0.4 and self._angle_sections['fright'] < 0.4:
                    state_description = 'case 11 - right and fright'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=1
                elif self._angle_sections['left'] < 0.4 and self._angle_sections['front'] > 0.4 and self._angle_sections['fleft'] < 0.4:
                    state_description = 'case 12 - left and fleft'
                    self._velocity.linear.x=0
                    self._velocity.angular.z=-1
                else:
                    state_description = 'case 1 - normal'
                    theta_z=euler_from_quaternion([self._current_orientation.x,self._current_orientation.y,self._current_orientation.z,self._current_orientation.w])[2]
                    angle=atan2(self._destination.y-self._current_position.y,self._destination.x-self._current_position.x)
                    if self._angle_sections['back'] < 0.4:
                            self._velocity.linear.x=0.15
                            self._velocity.angular.z=0.0
                    else:
                        if abs(angle-theta_z) > 0.1:
                            if angle<theta_z:
                                self._velocity.linear.x=0.0
                                self._velocity.angular.z=-0.3
                            else:
                                self._velocity.linear.x=0.0
                                self._velocity.angular.z=0.3
                        else:
                            self._velocity.linear.x=0.15
                            self._velocity.angular.z=0.0
                    rospy.loginfo('front={:.3f} m, fleft={:.3f} m, right={:.3f} m, fright={:.3f} m, left={:.3f} m'.format(self._angle_sections['front'],self._angle_sections['fleft'],self._angle_sections['right'],self._angle_sections['fright'],self._angle_sections['left']))
                rospy.loginfo(state_description)
            else:
                self._velocity.linear.x=0.0
                self._velocity.angular.z=0.0
                self._arrived_to_destination=True
                rospy.loginfo('El robot ha llegado a su destino')
        else:
            self._velocity.linear.x=0.0
            self._velocity.angular.z=0.0
            rospy.loginfo('El robot ha llegado a su destino')
    def publish_moved_robot(self):
        rospy.spin()
if __name__=='__main__':
    rospy.init_node('robot_movement_node')
    rob_mov=RobotMover()
    rob_mov.publish_moved_robot()