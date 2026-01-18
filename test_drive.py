import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class MovingRobot(Node):
    def __init__(self):
        super().__init__('moving_robot')
        
        # 1. Setup Wheel Publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 2. Setup Position Broadcaster (The "Mover")
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.angle = 0.0      # For wheel rotation
        self.path_angle = 0.0 # For robot driving in a circle
        self.x = 0.0
        self.y = 0.0

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        
        # --- PART A: SPIN THE WHEELS ---
        self.angle += 0.2
        joint_msg = JointState()
        joint_msg.header.stamp = current_time
        joint_msg.name = ['front_left_wheel_joint', 'front_right_wheel_joint', 
                          'back_left_wheel_joint', 'back_right_wheel_joint']
        joint_msg.position = [self.angle, self.angle, self.angle, self.angle]
        self.joint_pub.publish(joint_msg)

        # --- PART B: MOVE THE ROBOT (The TF) ---
        # We will drive in a circle
        self.path_angle += 0.02
        radius = 2.0
        
        # Calculate new X and Y position
        self.x = radius * math.cos(self.path_angle)
        self.y = radius * math.sin(self.path_angle)

        # Create the Transform message
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'     # The fixed world frame
        t.child_frame_id = 'base_link' # Your robot
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Calculate rotation (Quaternion) so robot faces the direction of travel
        # Facing direction is path_angle + 90 degrees (1.57 rad)
        yaw = self.path_angle + 1.57
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)

        # Send the move command to Rviz
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = MovingRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()