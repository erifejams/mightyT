import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import sys
#from timeit import Timer  #install

from enum import Enum


class RobotState(Enum):
    oneCircle = 1
    secondCircle = 2


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        ###STATE MACHINE
        self.current_state = RobotState.oneCircle
        self.next_state = RobotState.oneCircle
        self.timer = 1/60
        self.countTime = 0
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, '/thymio0/cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, '/thymio0/odom', self.odom_callback, 10)

        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
        
    def update_callback(self):
        # Let's just set some hard-coded velocities in this example
        cmd_vel = Twist() 
        

        #self.get_logger().warning("checking which circle I should draw")
        #self.vel_publisher.publish(cmd_vel)
        
        ##FIRST CIRCLE
        if self.current_state == RobotState.oneCircle:
            
            self.get_logger().info("drawing first circle")
            cmd_vel.linear.x  = 1.6
            cmd_vel.angular.z = -0.5 
            self.countTime += 1
            self.get_logger().info(f"{self.countTime}")
            # Publish the command
            self.vel_publisher.publish(cmd_vel)

            if self.countTime > 180:
                #turn the robot in another direction
                cmd_vel.angular.z = -2.2 #-1.8
                cmd_vel.linear.x  = 1.7
                

                ##updates to the second circle 
                self.next_state = RobotState.secondCircle
                
                  # Publish the command
                cmd_vel.linear.x  = 0.0
                self.vel_publisher.publish(cmd_vel)

        # ##SECOND CIRCLE
        elif self.next_state == RobotState.secondCircle: ##SECOND CIRCLE
            self.get_logger().info("drawing second circle")
            cmd_vel.linear.x  = 1.6
            cmd_vel.angular.z = -1.2
            self.countTime += 1

            # Publish the command
            self.vel_publisher.publish(cmd_vel)
            

            if self.countTime  > 190:
                ##changes the velocity to positive so it can do the first circle

                cmd_vel.angular.z = 1.2
                ##updates to the first circle 
                self.next_state = RobotState.oneCircle
                self.countTime = 0
            


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
