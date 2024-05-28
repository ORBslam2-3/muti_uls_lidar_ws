import rospy
from sensor_msgs.msg import Range
import numpy as np
import matplotlib.pyplot as plt

class UltrasonicArray:
    def __init__(self):
        rospy.init_node('ultrasonic_array', anonymous=True)
        self.sensor_data = {
            'uls_center': None,
            'uls_top': None,
            'uls_down': None,
            'uls_left': None,
            'uls_right': None
        }
        self.sensor_positions = {
            'uls_center': [0.20, 0, 0.5],
            'uls_top': [0.20, 0, 0.55],
            'uls_down': [0.20, 0, 0.45],
            'uls_left': [0.20, -0.05, 0.5],
            'uls_right': [0.20, 0.05, 0.5]
        }
        self.subscribers = [
            rospy.Subscriber('/uls_center_range', Range, self.callback, 'uls_center'),
            rospy.Subscriber('/uls_top_range', Range, self.callback, 'uls_top'),
            rospy.Subscriber('/uls_down_range', Range, self.callback, 'uls_down'),
            rospy.Subscriber('/uls_left_range', Range, self.callback, 'uls_left'),
            rospy.Subscriber('/uls_right_range', Range, self.callback, 'uls_right')
        ]

    def callback(self, data, sensor_id):
        self.sensor_data[sensor_id] = data.range

    def beamforming(self, focus_point):
        c = 340.0  # Speed of sound in m/s
        delays = [np.linalg.norm(np.array(pos) - np.array(focus_point)) / c for pos in self.sensor_positions.values()]
        adjusted_signals = [self.sensor_data[sensor] - delay for sensor, delay in zip(self.sensor_data.keys(), delays)]
        return sum(adjusted_signals) / len(adjusted_signals)

    def create_image(self):
        grid_size = 100
        image = np.zeros((grid_size, grid_size))

        for x in range(grid_size):
            for y in range(grid_size):
                focus_point = [x / grid_size, y / grid_size, 0.5]  # Example 2D grid with fixed z
                image[x, y] = self.beamforming(focus_point)

        plt.imshow(image, cmap='hot', interpolation='nearest')
        plt.colorbar()
        plt.title('Ultrasonic Array Beamforming Visualization')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.show()

    def process_data(self):
        # Transform sensor readings into a 2D plane for visualization
        x_positions = []
        y_positions = []

        for sensor, distance in self.sensor_data.items():
            if distance is not None:
                pos = self.sensor_positions[sensor]
                angle = np.arctan2(pos[1], pos[0])
                x = pos[0] + distance * np.cos(angle)
                y = pos[1] + distance * np.sin(angle)
                x_positions.append(x)
                y_positions.append(y)

        return x_positions, y_positions

    def visualize_contours(self):
        x_positions, y_positions = self.process_data()
        plt.scatter(x_positions, y_positions, c='r', marker='o')
        plt.plot(x_positions, y_positions, 'b-')
        plt.title('Obstacle Contour')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.grid(True)
        plt.show()

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if None not in self.sensor_data.values():
                self.visualize_contours()
            rate.sleep()

if __name__ == '__main__':
    try:
        array = UltrasonicArray()
        array.run()
    except rospy.ROSInterruptException:
        pass
