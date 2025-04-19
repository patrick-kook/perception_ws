#!/usr/bin/env python
import rospy
import os
from math import *
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from std_msgs.msg import Int8
import time
class F110:
    def __init__(self) :
        rospy.init_node("Joy_node")
        #self.pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size=10)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)
        self.speed_pub = rospy.Publisher("/commands/motor/duty_cycle", Float64, queue_size=10)
        self.nav_msg = AckermannDriveStamped()
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.flag = False
        self.speed = 0
        self.steer = 0.5
        rospy.Subscriber("/joy", Joy, self.joy_CB)
        #rospy.Subscriber("/key", Int8, self.keyboard_CB)
        self.joy_msg = Joy()
        

        

    def joy_CB(self, msg):
        #os.system("clear")
        self.joy_msg = msg
        if self.joy_msg.buttons[0] == 1:
            print("Drive")
 	
        
        
        
        self.speed = 0.05 * self.joy_msg.axes[1]
        self.steer = (-1*self.joy_msg.axes[2])/2 + 0.5
        #print(self.joy_msg.axes[0])
        #print(speed)
        #self.pub_speed_and_steer(speed,steer)

        
    def pub_speed_and_steer(self, speed, steer):
        self.steer_msg.data = steer
        self.speed_msg.data = speed
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)

    def run(self):
    	self.pub_speed_and_steer(self.speed,self.steer)

def main():
    try:
        f110 = F110()
        while not rospy.is_shutdown():
            f110.run()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
