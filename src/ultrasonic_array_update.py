import rospy
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import numpy as np
import open3d as o3d
import tf
from tf.transformations import quaternion_matrix

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
        self.current_pose = None

        self.subscribers = [
            rospy.Subscriber('/uls_center_range', Range, self.sensor_callback, 'uls_center'),
            rospy.Subscriber('/uls_top_range', Range, self.sensor_callback, 'uls_top'),
            rospy.Subscriber('/uls_down_range', Range, self.sensor_callback, 'uls_down'),
            rospy.Subscriber('/uls_left_range', Range, self.sensor_callback, 'uls_left'),
            rospy.Subscriber('/uls_right_range', Range, self.sensor_callback, 'uls_right')
        ]

        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)  # 等待TF变换发布

    def sensor_callback(self, data, sensor_id):
        self.sensor_data[sensor_id] = data.range
        if all(self.sensor_data.values()) and self.current_pose:
            self.update_point_cloud()
            self.sensor_data = {key: None for key in self.sensor_data}  # Reset sensor data for the next scan

    def odom_callback(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        self.current_pose = (position, orientation)

    def update_point_cloud(self):
        points = []

        for sensor, distance in self.sensor_data.items():
            if distance is not None and distance > 0:  # Ignore invalid measurements
                try:
                    (trans_sensor, rot_sensor) = self.tf_listener.lookupTransform('/base_link', f'/{sensor}_link', rospy.Time(0))
                    local_point = np.array([distance, 0, 0, 1.0])  # Distance is along the x-axis
                    trans_matrix_sensor = self.tf_listener.fromTranslationRotation(trans_sensor, rot_sensor)
                    point_in_base_link = np.dot(trans_matrix_sensor, local_point)
                    
                    # 获取当前位姿变换矩阵
                    position, orientation = self.current_pose
                    trans_matrix_base = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
                    trans_matrix_base[:3, 3] = [position.x, position.y, position.z]
                    global_point = np.dot(trans_matrix_base, point_in_base_link)
                    
                    points.append(global_point[:3])
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn(f"TF exception: {e}")
                    continue
        
        if points:
            self.point_cloud.extend(points)
            rospy.loginfo(f"Added {len(points)} points, total points: {len(self.point_cloud)}")
            self.save_point_cloud()

    def save_point_cloud(self):
        points = np.array(self.point_cloud)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud("output.pcd", pcd)
        rospy.loginfo("Saved point cloud to output.pcd")

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        array = UltrasonicArray()
        array.run()
    except rospy.ROSInterruptException:
        pass
