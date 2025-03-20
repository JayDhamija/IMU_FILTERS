from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import rclpy

class ImuToTransformNode(Node):
    def __init__(self):
        super().__init__('msgs_conversion')
        
        # Subscriber for IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/raw',  # Replace with your actual IMU topic name
            self.imu_callback,
            10
        )
        
        # Publisher for TransformStamped data on a different topic
        self.transform_publisher = self.create_publisher(
            TransformStamped,
            '/imu/transform',  # New topic for TransformStamped data
            10
        )

    def imu_callback(self, imu_msg):
        # Convert IMU to TransformStamped
        transform_msg = self.imu_to_transform_stamped(imu_msg)
        
        # Publish the TransformStamped message
        self.transform_publisher.publish(transform_msg)

    def imu_to_transform_stamped(self, imu_msg):
        # Create a TransformStamped message
        transform_msg = TransformStamped()
        
        # Copy the header from the IMU message
        transform_msg.header = imu_msg.header

        # Set orientation (quaternion) from IMU to the rotation in TransformStamped
        transform_msg.transform.rotation = imu_msg.orientation

        # Set translation to zero as IMU data typically does not contain positional data
        transform_msg.transform.translation.x = 0.0
        transform_msg.transform.translation.y = 0.0
        transform_msg.transform.translation.z = 0.0

        return transform_msg


def main(args=None):
    rclpy.init(args=args)
    node = ImuToTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
