import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion
from collections import deque
import time

from imu_filters.filters import MedianFilter, MeanFilter, LowPassFilter  # type: ignore

class ImuFilterNode(Node):
    def __init__(self):
        super().__init__('imu_filter_node')
        
        # Initialize filters with desired parameters
        self.median_filter = MedianFilter(window_size=5)
        self.mean_filter = MeanFilter(window_size=5)
        self.low_pass_filter = LowPassFilter(alpha=0.1)

        # Subscribe to the IMU data topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',  # Replace with your IMU topic if different
            self.imu_callback,
            10
        )

        # Initialize data queues for plotting
        self.time_data = deque(maxlen=100)
        self.yaw_raw_data = deque(maxlen=100)
        self.yaw_median_data = deque(maxlen=100)
        self.yaw_mean_data = deque(maxlen=100)
        self.yaw_lowpass_data = deque(maxlen=100)

        plt.ion()  # Enable interactive mode
        plt.show()

    def imu_callback(self, msg):
        # Extract quaternion and convert to Euler angles
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
        yaw_degrees = yaw * 180 / 3.14159  # Convert yaw to degrees

        current_time = time.time()
        
        # Apply each filter to the raw yaw data
        median_yaw = self.median_filter.apply(yaw_degrees)
        mean_yaw = self.mean_filter.apply(yaw_degrees)
        lowpass_yaw = self.low_pass_filter.apply(yaw_degrees)
        
        # Append raw and filtered data for plotting
        self.time_data.append(current_time)
        self.yaw_raw_data.append(yaw_degrees)
        self.yaw_median_data.append(median_yaw)
        self.yaw_mean_data.append(mean_yaw)
        self.yaw_lowpass_data.append(lowpass_yaw)

        # Plot the raw and filtered data
        plt.clf()
        plt.plot(self.time_data, self.yaw_raw_data, label="Raw Yaw", color="blue")
        plt.plot(self.time_data, self.yaw_median_data, label="Median Filtered Yaw", color="green")
        plt.plot(self.time_data, self.yaw_mean_data, label="Mean Filtered Yaw", color="orange")
        plt.plot(self.time_data, self.yaw_lowpass_data, label="Low-Pass Filtered Yaw", color="red")
        
        # Customize plot
        plt.xlabel("Time")
        plt.ylabel("Yaw angle (degrees)")
        plt.legend()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)

    imu_filter_node = ImuFilterNode()
    rclpy.spin(imu_filter_node)
    
    imu_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
