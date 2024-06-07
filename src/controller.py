#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from math import sqrt, atan2
from tf.transformations import euler_from_quaternion

class Controller:

    def __init__(self):
        rospy.init_node("controller", anonymous=True)

        # Velocity publishers
        self.volta1_pub = rospy.Publisher("/volta1/cmd_vel", Twist, queue_size=10)
        self.volta2_pub = rospy.Publisher("/volta2/cmd_vel", Twist, queue_size=10)
        self.volta3_pub = rospy.Publisher("/volta3/cmd_vel", Twist, queue_size=10)
        self.volta4_pub = rospy.Publisher("/volta4/cmd_vel", Twist, queue_size=10)
        rospy.loginfo("Publishers initialized")

        # Odometry subscriber
        rospy.Subscriber("/gazebo/model_states", ModelStates, callback=self.model_states_callback)
        rospy.loginfo("Subscriber initialized")

        # self.goal_points = [(3,3),(-1,7),(3,-3),(-1,-7),(0,0)]
        # self.goal_points = [(0,3),(3,0),(0,-3),(-3,0),(0,0)]
        self.goal_points = [(4,3),(-3,-5),(0,3),(4,0),(-3,-3)]
        self.volta_goal_points = []

        self.volta1_reached = False
        self.volta2_reached = False
        self.volta3_reached = False
        self.volta4_reached = False

        self.linear_speed = 0.3
        self.angular_speed = 0.2

        self.volta1_x = 0
        self.volta1_y = 0
        self.volta1_theta = 0

        self.volta2_x = 0
        self.volta2_y = 0
        self.volta2_theta = 0

        self.volta3_x = 0
        self.volta3_y = 0
        self.volta3_theta = 0

        self.volta4_x = 0
        self.volta4_y = 0
        self.volta4_theta = 0

        self.rate = rospy.Rate(10)

    def model_states_callback(self, data):
        for i in range(1,5):
            if data.name[i] == "volta1":
                self.volta1_x = data.pose[i].position.x
                self.volta1_y = data.pose[i].position.y
                _, _, self.volta1_theta = euler_from_quaternion([data.pose[i].orientation.x,
                                                   data.pose[i].orientation.y,
                                                   data.pose[i].orientation.z,
                                                   data.pose[i].orientation.w])
            elif data.name[i] == "volta2":
                self.volta2_x = data.pose[i].position.x
                self.volta2_y = data.pose[i].position.y
                _, _, self.volta2_theta = euler_from_quaternion([data.pose[i].orientation.x,
                                                   data.pose[i].orientation.y,
                                                   data.pose[i].orientation.z,
                                                   data.pose[i].orientation.w])
                
            elif data.name[i] == "volta3":
                self.volta3_x = data.pose[i].position.x
                self.volta3_y = data.pose[i].position.y
                _, _, self.volta3_theta = euler_from_quaternion([data.pose[i].orientation.x,
                                                   data.pose[i].orientation.y,
                                                   data.pose[i].orientation.z,
                                                   data.pose[i].orientation.w])
                
            elif data.name[i] == "volta4":
                self.volta4_x = data.pose[i].position.x
                self.volta4_y = data.pose[i].position.y
                _, _, self.volta4_theta = euler_from_quaternion([data.pose[i].orientation.x,
                                                   data.pose[i].orientation.y,
                                                   data.pose[i].orientation.z,
                                                   data.pose[i].orientation.w])
                
            else:
                rospy.logerr("VOLTA ODOM NOT FOUND")

    def euclidean_distance(self, x1, y1, x2, y2):
        return sqrt((x2-x1)**2+(y2-y1)**2)
    
    def calc_angle(self, x1, y1, x2, y2):
        return atan2(y2-y1,x2-x1)
    
    def calc_goal_points(self, index):
        goal_x, goal_y = self.goal_points[index]
        self.volta_goal_points = [(goal_x-1,goal_y+1),
                                  (goal_x-1,goal_y-1),
                                  (goal_x+1,goal_y+1),
                                  (goal_x+1,goal_y-1)]
        
    def go_to_goal(self, id):
        if id == 1:
            heading_a1 = self.calc_angle(self.volta1_x, self.volta1_y, self.volta_goal_points[id-1][0], self.volta_goal_points[id-1][1])
            a1 = heading_a1 - self.volta1_theta
            e1 = self.euclidean_distance(self.volta1_x, self.volta1_y, self.volta_goal_points[id-1][0], self.volta_goal_points[id-1][1])
            volta1_vel = Twist()
            if abs(a1)>0.1:
                volta1_vel.angular.z = self.angular_speed * a1 / abs(a1)
            else:
                volta1_vel.angular.z = 0
            if e1>0.1:
                volta1_vel.linear.x = self.linear_speed
            else:
                volta1_vel.linear.x = 0
                volta1_vel.angular.z = 0
                self.volta1_reached = True
            self.volta1_pub.publish(volta1_vel)
        
        if id == 2:
            heading_a2 = self.calc_angle(self.volta2_x, self.volta2_y, self.volta_goal_points[id-1][0], self.volta_goal_points[id-1][1])
            a2 = heading_a2 - self.volta2_theta
            e2 = self.euclidean_distance(self.volta2_x, self.volta2_y, self.volta_goal_points[id-1][0], self.volta_goal_points[id-1][1])
            volta2_vel = Twist()
            if abs(a2)>0.1:
                volta2_vel.angular.z = self.angular_speed * a2 / abs(a2)
            else:
                volta2_vel.angular.z = 0
            if e2>0.1:
                volta2_vel.linear.x = self.linear_speed
            else:
                volta2_vel.linear.x = 0
                volta2_vel.angular.z = 0
                self.volta2_reached = True
            self.volta2_pub.publish(volta2_vel)

        if id == 3:
            heading_a3 = self.calc_angle(self.volta3_x, self.volta3_y, self.volta_goal_points[id-1][0], self.volta_goal_points[id-1][1])
            a3 = heading_a3 - self.volta3_theta
            e3 = self.euclidean_distance(self.volta3_x, self.volta3_y, self.volta_goal_points[id-1][0], self.volta_goal_points[id-1][1])
            volta3_vel = Twist()
            if abs(a3)>0.1:
                volta3_vel.angular.z = self.angular_speed * a3 / abs(a3)
            else:
                volta3_vel.angular.z = 0
            if e3>0.1:
                volta3_vel.linear.x = self.linear_speed
            else:
                volta3_vel.linear.x = 0
                volta3_vel.angular.z = 0
                self.volta3_reached = True
            self.volta3_pub.publish(volta3_vel)

        if id == 4:
            heading_a4 = self.calc_angle(self.volta4_x, self.volta4_y, self.volta_goal_points[id-1][0], self.volta_goal_points[id-1][1])
            a4 = heading_a4 - self.volta4_theta
            e4 = self.euclidean_distance(self.volta4_x, self.volta4_y, self.volta_goal_points[id-1][0], self.volta_goal_points[id-1][1])
            volta4_vel = Twist()
            if abs(a4)>0.1:
                volta4_vel.angular.z = self.angular_speed * a4 / abs(a4)
            else:
                volta4_vel.angular.z = 0
            if e4>0.1:
                volta4_vel.linear.x = self.linear_speed
            else:
                volta4_vel.linear.x = 0
                volta4_vel.angular.z = 0
                self.volta4_reached = True
            self.volta4_pub.publish(volta4_vel)
        
    def start_sequence(self):
        for i in range(0,5):
            rospy.loginfo(self.goal_points[i])
            self.calc_goal_points(i)
            rospy.loginfo(self.volta_goal_points)

            self.volta1_reached = False
            self.volta2_reached = False
            self.volta3_reached = False
            self.volta4_reached = False

            while((not self.volta1_reached) or (not self.volta2_reached) or (not self.volta3_reached) or (not self.volta4_reached)):
                self.go_to_goal(1)
                self.go_to_goal(2)
                self.go_to_goal(3)
                self.go_to_goal(4)

                self.rate.sleep()

            rospy.loginfo("Goal reached")
            rospy.sleep(5)

def main():
    controller = Controller()
    controller.start_sequence()

if __name__ == "__main__":
    main()