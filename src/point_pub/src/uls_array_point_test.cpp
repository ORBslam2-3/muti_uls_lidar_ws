#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>

class UltrasonicArray {
public:
    UltrasonicArray() {
        ros::NodeHandle nh;

        // 初始化传感器位置
        sensor_positions = {
            {"uls_11", {0.20, -0.20, 0.7}},
            {"uls_12", {0.20, -0.1, 0.7}},
            {"uls_13", {0.20, 0, 0.7}},
            {"uls_14", {0.20, 0.1, 0.7}},
            {"uls_15", {0.20, 0.2, 0.7}},
            {"uls_21", {0.20, -0.20, 0.6}},
            {"uls_22", {0.20, -0.1, 0.6}},
            {"uls_23", {0.20, 0, 0.6}},
            {"uls_24", {0.20, 0.1, 0.6}},
            {"uls_25", {0.20, 0.2, 0.6}},
            {"uls_31", {0.20, -0.20, 0.5}},
            {"uls_32", {0.20, -0.1, 0.5}},
            {"uls_33", {0.20, 0, 0.5}},
            {"uls_34", {0.20, 0.1, 0.5}},
            {"uls_35", {0.20, 0.2, 0.5}},
            {"uls_41", {0.20, -0.20, 0.4}},
            {"uls_42", {0.20, -0.1, 0.4}},
            {"uls_43", {0.20, 0, 0.4}},
            {"uls_44", {0.20, 0.1, 0.4}},
            {"uls_45", {0.20, 0.2, 0.4}},
            {"uls_51", {0.20, -0.20, 0.3}},
            {"uls_52", {0.20, -0.1, 0.3}},
            {"uls_53", {0.20, 0, 0.3}},
            {"uls_54", {0.20, 0.1, 0.3}},
            {"uls_55", {0.20, 0.2, 0.3}}
        };

        // 订阅传感器话题
        for (const auto& kv : sensor_positions) {
            subscribers.push_back(nh.subscribe<sensor_msgs::Range>(
                "/" + kv.first + "_range", 10, boost::bind(&UltrasonicArray::callback, this, _1, kv.first)));
        }

        // 订阅里程计数据
        odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &UltrasonicArray::odom_callback, this);

        // 发布点云
        point_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud", 1);
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        // 初始化 filter_point 为 false
        filter_point = false;

    }

    void run() {
        ros::Rate rate(1);  // 1 Hz
        while (ros::ok()) {
            ros::spinOnce();
            if (all_data_received()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                get_point_cloud(cloud);
                // filter_ground_points(cloud);
                // cluster_and_visualize(cloud);
            }
            rate.sleep();
        }
    }

private:
    std::map<std::string, float> sensor_data;
    std::map<std::string, std::vector<float>> sensor_positions;
    std::vector<ros::Subscriber> subscribers;
    ros::Subscriber odom_sub;
    ros::Publisher point_cloud_pub;
    ros::Publisher marker_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud{new pcl::PointCloud<pcl::PointXYZ>};
    tf::TransformListener tf_listener;
    tf::StampedTransform current_transform;
    bool initial_transform_set = false;
    Eigen::Isometry3d initial_transform = Eigen::Isometry3d::Identity();
    // 新增的成员变量
    bool filter_point;

    void callback(const sensor_msgs::Range::ConstPtr& msg, const std::string& sensor_id) {
        sensor_data[sensor_id] = msg->range;
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        try {
            tf_listener.lookupTransform("odom", "base_link", ros::Time(0), current_transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    bool all_data_received() {
        for (const auto& kv : sensor_positions) {
            if (sensor_data.find(kv.first) == sensor_data.end()) {
                return false;
            }
        }
        return true;
    }

    void get_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        bool filter_all_points = false;

        // 先检查是否满足条件1和条件2
        for (const auto& kv : sensor_data) {
            const std::string& sensor = kv.first;
            float distance = kv.second;

            if (sensor == "uls_11" && distance > 3.20 && distance < 3.30) {
                for (const auto& inner_kv : sensor_data) {
                    const std::string& inner_sensor = inner_kv.first;
                    float inner_distance = inner_kv.second;
                    if (inner_sensor == "uls_21" && inner_distance > 2.80 && inner_distance < 2.90) {
                        filter_all_points = true;
                        break;
                    }
                }
            }
            if (sensor == "uls_45" && distance > 2.00 && distance < 2.10) {
                for (const auto& inner_kv : sensor_data) {
                    const std::string& inner_sensor = inner_kv.first;
                    float inner_distance = inner_kv.second;
                    if (inner_sensor == "uls_55" && inner_distance > 1.60 && inner_distance < 1.70) {
                        filter_all_points = true;
                        break;
                    }
                }
            }
            
            if (filter_all_points) {
                break;
            }
        }

        // 如果满足条件1和条件2，则跳过所有点
        if (filter_all_points) {
            std::cout << "所有点被滤除" << std::endl;
            return;
        }


        std::cout<<"无障碍物情况下，这个地方不会运行到，因为在前边就会return"<<std::endl;
        // 否则，生成点云
        for (const auto& kv : sensor_data) {
            const std::string& sensor = kv.first;
            float distance = kv.second;
            
            const std::vector<float>& pos = sensor_positions[sensor];
            float angle = std::atan2(pos[1], pos[0]);
            // float x = pos[0] + distance * std::cos(angle);
            // float y = pos[1] + distance * std::sin(angle);
            // float z = pos[2];
            float x = pos[0] + distance ;
            float y = pos[1] ;
            float z = pos[2];

            cloud->points.push_back(pcl::PointXYZ(x, y, z));
        }

        // 将当前点云转换到全局坐标系
        Eigen::Isometry3d transform;
        tf::transformTFToEigen(current_transform, transform);
        if (!initial_transform_set) {
            initial_transform = transform;
            initial_transform_set = true;
        }

        Eigen::Isometry3d relative_transform = initial_transform.inverse() * transform;
        pcl::transformPointCloud(*cloud, *cloud, relative_transform.matrix());

        *accumulated_cloud += *cloud;

        // 发布累积点云
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*accumulated_cloud, output);
        output.header.frame_id = "odom";
        point_cloud_pub.publish(output);
    }

    // void get_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    //     for (const auto& kv : sensor_data) {
    //         const std::string& sensor = kv.first;
    //         float distance = kv.second;
    //         // 过滤地面点
    //         if (sensor == "uls_11" && distance > 3.2 && distance < 3.3) {
    //             std::cout<<"满足条件1"<<std::endl;
    //             for (const auto& inner_kv : sensor_data) {
    //                 const std::string& inner_sensor = inner_kv.first;
    //                 float inner_distance = inner_kv.second;
    //                 if (inner_sensor == "uls_21") {
    //                     std::cout<<"uls_21:"<<inner_kv.second<<std::endl;
    //                 }
    //                 if (inner_sensor == "uls_21" && inner_distance > 2.85 && inner_distance < 2.88) {
    //                     std::cout<<"满足条件2"<<std::endl;
    //                     filter_point = true;
    //                     break;  // 找到一个匹配，标记为需要过滤并跳出内部循环
    //                 }
    //             }
    //         }
    //         std::cout<<"filter_point:"<<filter_point<<std::endl;
    //         if (filter_point) {
    //             filter_point = false;  // 重置标志位
    //             std::cout<<"生效，已进行滤除操作"<<std::endl;
    //             continue;  // 跳过当前点，不加入点云
    //         }
            
    //         const std::vector<float>& pos = sensor_positions[sensor];
    //         float angle = std::atan2(pos[1], pos[0]);
    //         float x = pos[0] + distance * std::cos(angle);
    //         float y = pos[1] + distance * std::sin(angle);
    //         float z = pos[2];

    //         cloud->points.push_back(pcl::PointXYZ(x, y, z));
    //     }

    //     // 将当前点云转换到全局坐标系
    //     Eigen::Isometry3d transform;
    //     tf::transformTFToEigen(current_transform, transform);
    //     if (!initial_transform_set) {
    //         initial_transform = transform;
    //         initial_transform_set = true;
    //     }

    //     Eigen::Isometry3d relative_transform = initial_transform.inverse() * transform;
    //     pcl::transformPointCloud(*cloud, *cloud, relative_transform.matrix());

    //     *accumulated_cloud += *cloud;

    //     // 发布累积点云
    //     sensor_msgs::PointCloud2 output;
    //     pcl::toROSMsg(*accumulated_cloud, output);
    //     output.header.frame_id = "odom";
    //     point_cloud_pub.publish(output);
    // }

    // void filter_ground_points(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    //     pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    //     pcl::SACSegmentation<pcl::PointXYZ> seg;
    //     seg.setOptimizeCoefficients(true);
    //     seg.setModelType(pcl::SACMODEL_PLANE);
    //     seg.setMethodType(pcl::SAC_RANSAC);
    //     seg.setDistanceThreshold(0.01);

    //     seg.setInputCloud(cloud);
    //     seg.segment(*inliers, *coefficients);

    //     pcl::ExtractIndices<pcl::PointXYZ> extract;
    //     extract.setInputCloud(cloud);
    //     extract.setIndices(inliers);
    //     extract.setNegative(true);
    //     extract.filter(*cloud_filtered);

    //     cloud = cloud_filtered;
    // }

    void cluster_and_visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // 创建KDTree对象来进行搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.1);  // 设置近邻搜索的搜索半径
        ec.setMinClusterSize(2);      // 设置一个聚类需要的最少点数目
        ec.setMaxClusterSize(25000);  // 设置一个聚类需要的最大点数目
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 可视化聚类结果
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "ultrasonic_array";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        std::vector<std_msgs::ColorRGBA> colors = generate_colors(cluster_indices.size());

        int cluster_id = 0;
        for (const auto& indices : cluster_indices) {
            for (const auto& idx : indices.indices) {
                const pcl::PointXYZ& point = cloud->points[idx];

                std_msgs::ColorRGBA color = colors[cluster_id];
                marker.colors.push_back(color);

                geometry_msgs::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                marker.points.push_back(p);
            }
            cluster_id++;
        }

        marker_pub.publish(marker);
    }

    // 颜色生成函数的示例实现
    std::vector<std_msgs::ColorRGBA> generate_colors(int num_colors) {
        std::vector<std_msgs::ColorRGBA> colors;
        for (int i = 0; i < num_colors; ++i) {
            std_msgs::ColorRGBA color;
            color.r = static_cast<float>(rand()) / RAND_MAX;
            color.g = static_cast<float>(rand()) / RAND_MAX;
            color.b = static_cast<float>(rand()) / RAND_MAX;
            color.a = 1.0;
            colors.push_back(color);
        }
        return colors;
    }

};

int main(int argc, char** argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "ultrasonic_array");

    UltrasonicArray array;
    array.run();

    return 0;
}   