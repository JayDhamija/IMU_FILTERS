import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import time

class ButterworthFilterTypeComparison(Node):
    def __init__(self, cutoff=0.5, sample_rate=50):
        super().__init__('butterworth_filter_type_comparison')
        
        self.sample_rate = sample_rate
        self.cutoff = cutoff
        self.order = 4
        self.window_size = 100
        
        self.raw_data = deque(maxlen=self.window_size)
        self.lowpass_data = deque(maxlen=self.window_size)
        self.highpass_data = deque(maxlen=self.window_size)
        self.bandpass_data = deque(maxlen=self.window_size)
        self.bandstop_data = deque(maxlen=self.window_size)
        
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',  # Use your actual IMU data topic
            self.imu_callback,
            10
        )
        
        plt.ion()
        plt.show()

    def butterworth_filter(self, data, filter_type):
        nyquist = 0.5 * self.sample_rate
        if filter_type == "lowpass":
            b, a = butter(self.order, self.cutoff / nyquist, btype='low')
        elif filter_type == "highpass":
            b, a = butter(self.order, self.cutoff / nyquist, btype='high')
        elif filter_type == "bandpass":
            b, a = butter(self.order, np.array([0.1, self.cutoff]) / nyquist, btype='band')
        elif filter_type == "bandstop":
            b, a = butter(self.order, np.array([0.1, self.cutoff]) / nyquist, btype='bandstop')
        return filtfilt(b, a, data)

    
    def imu_callback(self, msg):
        # Extract the data directly from the IMU message
        data_array = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        # Only apply the filter when there are enough data points in the window
        if len(self.raw_data) > self.order:
            # Append filtered data to the lists
            self.lowpass_data.append(self.butterworth_filter(data_array, "lowpass"))
            self.highpass_data.append(self.butterworth_filter(data_array, "highpass"))
            self.bandpass_data.append(self.butterworth_filter(data_array, "bandpass"))
            self.bandstop_data.append(self.butterworth_filter(data_array, "bandstop"))
        else:
            # If not enough data, just append the raw data without filtering
            self.lowpass_data.append(data_array)
            self.highpass_data.append(data_array)
            self.bandpass_data.append(data_array)
            self.bandstop_data.append(data_array)
        
        # Ensure that self.raw_data also gets updated
        self.raw_data.append(data_array)  # Assuming data_array is the raw data

        # Plot the data
        self.plot_data()



    def plot_data(self):
        plt.clf()
        
        # Ensure the time_series length matches the filtered data length
        time_series = np.arange(len(self.raw_data))  # Assuming raw_data grows with each message
        
        # Plot the filtered data
        plt.plot(time_series, self.raw_data, label="Raw", color="gray")
        plt.plot(time_series, self.lowpass_data, label="Low-Pass", color="blue")
        plt.plot(time_series, self.highpass_data, label="High-Pass", color="red")
        plt.plot(time_series, self.bandpass_data, label="Band-Pass", color="green")
        plt.plot(time_series, self.bandstop_data, label="Band-Stop", color="orange")
        
        plt.xlabel("Time")
        plt.ylabel("Filtered Value")
        plt.legend()
        plt.pause(0.01)



def main(args=None):
    rclpy.init(args=args)
    node = ButterworthFilterTypeComparison()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
