import rospy
from sensor_msgs.msg import Range
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import DBSCAN
import tf

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
        self.point_cloud = []
        self.tf_listener = tf.TransformListener()

        self.subscribers = [
            rospy.Subscriber('/uls_center_range', Range, self.callback, 'uls_center'),
            rospy.Subscriber('/uls_top_range', Range, self.callback, 'uls_top'),
            rospy.Subscriber('/uls_down_range', Range, self.callback, 'uls_down'),
            rospy.Subscriber('/uls_left_range', Range, self.callback, 'uls_left'),
            rospy.Subscriber('/uls_right_range', Range, self.callback, 'uls_right')
        ]

    def callback(self, data, sensor_id):
        self.sensor_data[sensor_id] = data.range
        if all(self.sensor_data.values()):
            self.update_point_cloud()
            self.sensor_data = {key: None for key in self.sensor_data}  # Reset sensor data for the next scan

    def update_point_cloud(self):
        points = []

        for sensor, distance in self.sensor_data.items():
            if distance is not None and distance > 0:  # Ignore invalid measurements
                try:
                    (trans, rot) = self.tf_listener.lookupTransform('/base_link', f'/{sensor}_link', rospy.Time(0))
                    trans_matrix = self.tf_listener.fromTranslationRotation(trans, rot)

                    pos = self.sensor_positions[sensor]
                    local_point = np.array([pos[0] + distance, pos[1], pos[2], 1.0])
                    global_point = np.dot(trans_matrix, local_point)[:3]

                    points.append(global_point)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn(f"TF exception: {e}")
                    continue
        
        if points:
            self.point_cloud.extend(points)
            rospy.loginfo(f"Added {len(points)} points, total points: {len(self.point_cloud)}")

    def cluster_points(self, points):
        if len(points) == 0:
            return []
        clustering = DBSCAN(eps=0.1, min_samples=2).fit(points)
        labels = clustering.labels_
        return labels

    def visualize_clusters(self, points, labels):
        if len(points) == 0:
            rospy.logwarn("No points to visualize")
            return

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        unique_labels = set(labels)
        colors = plt.cm.Spectral(np.linspace(0, 1, len(unique_labels)))

        for k, col in zip(unique_labels, colors):
            if k == -1:
                col = 'k'  # Black used for noise.

            class_member_mask = (labels == k)

            xyz = points[class_member_mask]
            ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=col, marker='o')

        ax.set_title('3D Point Cloud Clustering')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        plt.show()

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if len(self.point_cloud) > 0:
                points = np.array(self.point_cloud)
                labels = self.cluster_points(points)
                self.visualize_clusters(points, labels)
            rate.sleep()

if __name__ == '__main__':
    try:
        array = UltrasonicArray()
        array.run()
    except rospy.ROSInterruptException:
        pass
