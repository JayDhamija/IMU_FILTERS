import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from tf_transformations import euler_from_quaternion

class ButterworthFilterOrderComparison(Node):
    def __init__(self, cutoff=0.5, sample_rate=50):
        super().__init__('butterworth_filter_order_comparison')
        
        self.sample_rate = sample_rate
        self.cutoff = cutoff
        self.window_size = 100
        
        # Initialize queues to store filtered data for different filter orders
        self.raw_yaw_data = deque(maxlen=self.window_size)
        self.order1_data = deque(maxlen=self.window_size)
        self.order2_data = deque(maxlen=self.window_size)
        self.order3_data = deque(maxlen=self.window_size)
        self.order4_data = deque(maxlen=self.window_size)
        
        # Subscription to IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',  # Replace with your IMU topic if different
            self.imu_callback,
            10
        )

        # Set up the plot
        self.fig, self.ax = plt.subplots()
        self.line_raw, = self.ax.plot([], [], label="Raw Yaw", color="gray")
        self.line_order1, = self.ax.plot([], [], label="1st Order Filter", color="blue")
        self.line_order2, = self.ax.plot([], [], label="2nd Order Filter", color="red")
        self.line_order3, = self.ax.plot([], [], label="3rd Order Filter", color="green")
        self.line_order4, = self.ax.plot([], [], label="4th Order Filter", color="purple")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Yaw Angle (Degrees)")
        self.ax.legend(loc="upper right")
        
        # Enable interactive plotting
        plt.ion()
        plt.show()

    def butterworth_filter(self, data, order):
        # Set up a Butterworth filter with specified order and cutoff frequency
        nyquist = 0.5 * self.sample_rate
        b, a = butter(order, self.cutoff / nyquist, btype='low')
        return filtfilt(b, a, data)
    
    def imu_callback(self, msg):
        # Convert quaternion to Euler angles to extract yaw
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
        yaw_degrees = yaw * 180 / np.pi  # Convert yaw to degrees
        self.raw_yaw_data.append(yaw_degrees)
        
        # Apply filters once we have enough data points
        if len(self.raw_yaw_data) == self.window_size:
            data_array = np.array(self.raw_yaw_data)
            self.order1_data.append(self.butterworth_filter(data_array, 1)[-1])
            self.order2_data.append(self.butterworth_filter(data_array, 2)[-1])
            self.order3_data.append(self.butterworth_filter(data_array, 3)[-1])
            self.order4_data.append(self.butterworth_filter(data_array, 4)[-1])
            self.update_plot()

    def update_plot(self):
        # Ensure all data lists are the same length before plotting
        if len(self.raw_yaw_data) == len(self.order1_data) == len(self.order2_data) == len(self.order3_data) == len(self.order4_data):
            time_series = range(len(self.raw_yaw_data))  # Use indices as the x-axis

            self.line_raw.set_data(time_series, list(self.raw_yaw_data))
            self.line_order1.set_data(time_series, list(self.order1_data))
            self.line_order2.set_data(time_series, list(self.order2_data))
            self.line_order3.set_data(time_series, list(self.order3_data))
            self.line_order4.set_data(time_series, list(self.order4_data))

            # Adjust plot limits
            self.ax.relim()
            self.ax.autoscale_view()
            
            # Refresh the plot without blocking the event loop
            plt.draw()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = ButterworthFilterOrderComparison()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
