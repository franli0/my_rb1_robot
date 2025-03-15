#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from my_rb1_ros.srv import Rotate, RotateResponse

class RotateRobotService:
    def __init__(self):
        self.service = rospy.Service('/rotate_robot', Rotate, self.handle_rotate_robot)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Robot state variables
        self.current_yaw = 0.0
        self.is_rotating = False
        self.target_yaw = 0.0
        self.initial_yaw = 0.0
        self.rotate_clockwise = False
        
        # Constants
        self.angular_speed = 0.3  # radians/second
        self.angular_tolerance = 0.03  # radians
        
        rospy.loginfo("Service Ready")

    def odom_callback(self, msg):
        # Extract orientation quaternion from odometry message
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)
        
        # Normalize yaw to be between -pi and pi
        self.current_yaw = self.normalize_angle(self.current_yaw)
        
        # If currently executing a rotation, check if we've reached the target
        if self.is_rotating:
            self.check_rotation_complete()

    def normalize_angle(self, angle):
        # Normalize angle to be between -pi and pi
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def check_rotation_complete(self):
        # Calculate the angular difference between current and target yaw
        error = self.normalize_angle(self.target_yaw - self.current_yaw)
        
        # If we're close enough to the target angle, stop rotating
        if abs(error) < self.angular_tolerance:
            self.stop_robot()
            self.is_rotating = False
            rospy.loginfo("Rotation complete! Final orientation: %.2f radians", self.current_yaw)
            return True
        
        # Otherwise, continue rotating
        cmd_vel = Twist()
        
        # Adjust angular velocity based on the direction we need to rotate
        if self.rotate_clockwise:
            cmd_vel.angular.z = -self.angular_speed
        else:
            cmd_vel.angular.z = self.angular_speed
            
        self.cmd_vel_pub.publish(cmd_vel)
        return False

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        
        # Send the stop command multiple times to ensure it's received
        for _ in range(3):
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.1)

    def handle_rotate_robot(self, req):
        if self.is_rotating:
            return RotateResponse(result="Error: Robot is already rotating")
        
        rospy.loginfo("Service Requested: Rotate %d degrees", req.degrees)
        
        # Convert degrees to radians
        angle_radians = math.radians(req.degrees)
        
        # Determine rotation direction based on the angle sign
        self.rotate_clockwise = angle_radians < 0
        
        # Record initial orientation
        self.initial_yaw = self.current_yaw
        
        # Calculate target yaw
        self.target_yaw = self.normalize_angle(self.initial_yaw + angle_radians)
        
        # Set rotating flag
        self.is_rotating = True
        
        # Start the rotation
        cmd_vel = Twist()
        if self.rotate_clockwise:
            cmd_vel.angular.z = -self.angular_speed
        else:
            cmd_vel.angular.z = self.angular_speed
            
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Wait until rotation is complete
        rate = rospy.Rate(10)  # 10 Hz
        while self.is_rotating and not rospy.is_shutdown():
            rate.sleep()
        
        rospy.loginfo("Service Completed")
        return RotateResponse(result="Success: Rotated %d degrees" % req.degrees)

if __name__ == '__main__':
    rospy.init_node('rotate_robot_service')
    
    service = RotateRobotService()
    
    rospy.spin()