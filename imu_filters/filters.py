# imu_filters/imu_filters/filters.py

from collections import deque
import numpy as np

class MedianFilter:
    def __init__(self, window_size=5):
        self.window = deque(maxlen=window_size)
    
    def apply(self, value):
        self.window.append(value)
        return np.median(self.window)

class MeanFilter:
    def __init__(self, window_size=5):
        self.window = deque(maxlen=window_size)
    
    def apply(self, value):
        self.window.append(value)
        return np.mean(self.window)

class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.prev_value = None
    
    def apply(self, value):
        if self.prev_value is None:
            self.prev_value = value
        filtered_value = self.alpha * value + (1 - self.alpha) * self.prev_value
        self.prev_value = filtered_value
        return filtered_value
