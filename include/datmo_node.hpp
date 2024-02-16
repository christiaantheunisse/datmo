#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <datmo/msg/track_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "cluster.hpp"

// Declare here

using namespace std;

class Cluster;

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;

class DatmoNode : public rclcpp::Node {
   public:
    // Constructor
    DatmoNode();

    void callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr&);
    void parameter_timer_callback();  // Added myself to handle parameters
    void Clustering(const sensor_msgs::msg::LaserScan::ConstSharedPtr&, vector<pointList>&);
    void transformPointList(const pointList&, pointList&);
    void visualiseGroupedPoints(const vector<pointList>&);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

   private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array;
    rclcpp::Publisher<datmo::msg::TrackArray>::SharedPtr pub_tracks_box_kf;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;

    sensor_msgs::msg::LaserScan scan;
    vector<Cluster> clusters;

    // Tuning Parameteres
    double dt;
    rclcpp::Time time;

    // initialised as one, because 0 index take the msgs that fail to be initialized
    unsigned long int cg = 1;         // group counter to be used as id of the clusters
    unsigned long int cclusters = 1;  // counter for the cluster objects to be used as id for the markers

    // Parameters
    double dth;
    double euclidean_distance;
    int max_cluster_size;
    bool p_marker_pub;
    bool p_min_marker_pub;
    bool w_exec_times;
    string lidar_frame;
    string world_frame;
    string lidar_topic;

    rclcpp::TimerBase::SharedPtr timer;
};