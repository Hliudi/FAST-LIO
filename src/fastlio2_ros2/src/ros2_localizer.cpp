// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// ROS2 (Humble) front-only FAST-LIO2 localization runtime for Astribot S1.
// Line-by-line port of cpp/ros1_localizer.cpp (dimos ref). The FAST-LIO core
// (custom_messages, FastLio) is unchanged; only the ROS I/O layer is ROS2.
//
// Publishes the Phase 3 localization contract:
//   - /odom (nav_msgs/Odometry)
//   - /tf   (odom -> base_link)
//
// ROS2 vs ROS1 差异（见 nav2-buildup-guide.md T3.2 表）:
//   - 继承 rclcpp::Node；declare_parameter / create_publisher / create_subscription
//   - ROS2 消息 header 没有 seq 字段 -> 用本地计数器/0 代替
//   - livox_ros_driver2::CustomMsg -> livox_interfaces::msg::CustomMsg
//   - 主循环用 rclcpp::spin_some + fastlio_->process()

#include <boost/make_shared.hpp>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "fast_lio.hpp"

namespace {

std::atomic<bool> g_running{true};

double ros_time_to_sec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) / 1e9;
}

// ---- 以下三个转换辅助函数与 ROS1 版逐字对应（只改输入消息类型 / 去掉 seq）----

custom_messages::ImuConstPtr to_fastlio_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    auto imu_msg = boost::make_shared<custom_messages::Imu>();

    imu_msg->header.seq = 0;   // ROS2 无 seq
    imu_msg->header.stamp = custom_messages::Time().fromSec(ros_time_to_sec(msg->header.stamp));
    imu_msg->header.frame_id = msg->header.frame_id;

    imu_msg->orientation.x = msg->orientation.x;
    imu_msg->orientation.y = msg->orientation.y;
    imu_msg->orientation.z = msg->orientation.z;
    imu_msg->orientation.w = msg->orientation.w;
    std::memcpy(
        imu_msg->orientation_covariance,
        msg->orientation_covariance.data(),
        sizeof(imu_msg->orientation_covariance)
    );

    imu_msg->angular_velocity.x = msg->angular_velocity.x;
    imu_msg->angular_velocity.y = msg->angular_velocity.y;
    imu_msg->angular_velocity.z = msg->angular_velocity.z;
    std::memcpy(
        imu_msg->angular_velocity_covariance,
        msg->angular_velocity_covariance.data(),
        sizeof(imu_msg->angular_velocity_covariance)
    );

    imu_msg->linear_acceleration.x = msg->linear_acceleration.x;
    imu_msg->linear_acceleration.y = msg->linear_acceleration.y;
    imu_msg->linear_acceleration.z = msg->linear_acceleration.z;
    std::memcpy(
        imu_msg->linear_acceleration_covariance,
        msg->linear_acceleration_covariance.data(),
        sizeof(imu_msg->linear_acceleration_covariance)
    );

    return imu_msg;
}

custom_messages::CstMsgConstPtr to_fastlio_lidar(const livox_interfaces::msg::CustomMsg::ConstSharedPtr& msg) {
    auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();

    lidar_msg->header.seq = 0;   // ROS2 无 seq
    lidar_msg->header.stamp = custom_messages::Time().fromSec(ros_time_to_sec(msg->header.stamp));
    lidar_msg->header.frame_id = msg->header.frame_id;
    lidar_msg->timebase = msg->timebase;
    lidar_msg->point_num = msg->point_num;
    lidar_msg->lidar_id = msg->lidar_id;
    for (int i = 0; i < 3; ++i) {
        lidar_msg->rsvd[i] = msg->rsvd[i];
    }

    lidar_msg->points.reserve(msg->points.size());
    for (const auto& point : msg->points) {
        custom_messages::CustomPoint converted;
        converted.offset_time = point.offset_time;
        converted.x = point.x;
        converted.y = point.y;
        converted.z = point.z;
        converted.reflectivity = point.reflectivity;
        converted.tag = point.tag;
        converted.line = point.line;
        lidar_msg->points.push_back(converted);
    }

    return lidar_msg;
}

nav_msgs::msg::Odometry to_ros_odometry(
    const custom_messages::Odometry& odom,
    const std::string& frame_id,
    const std::string& child_frame_id
) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = static_cast<int32_t>(odom.header.stamp.sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>(odom.header.stamp.nsecs);
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;

    msg.pose.pose.position.x = odom.pose.pose.position.x;
    msg.pose.pose.position.y = odom.pose.pose.position.y;
    msg.pose.pose.position.z = odom.pose.pose.position.z;
    msg.pose.pose.orientation.x = odom.pose.pose.orientation.x;
    msg.pose.pose.orientation.y = odom.pose.pose.orientation.y;
    msg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
    msg.pose.pose.orientation.w = odom.pose.pose.orientation.w;
    std::memcpy(msg.pose.covariance.data(), odom.pose.covariance, sizeof(odom.pose.covariance));

    msg.twist.twist.linear.x = odom.twist.twist.linear.x;
    msg.twist.twist.linear.y = odom.twist.twist.linear.y;
    msg.twist.twist.linear.z = odom.twist.twist.linear.z;
    msg.twist.twist.angular.x = odom.twist.twist.angular.x;
    msg.twist.twist.angular.y = odom.twist.twist.angular.y;
    msg.twist.twist.angular.z = odom.twist.twist.angular.z;
    std::memcpy(msg.twist.covariance.data(), odom.twist.covariance, sizeof(odom.twist.covariance));

    return msg;
}

geometry_msgs::msg::TransformStamped to_odom_tf(
    const nav_msgs::msg::Odometry& odom,
    const std::string& frame_id,
    const std::string& child_frame_id
) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.header.frame_id = frame_id;
    transform.child_frame_id = child_frame_id;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    return transform;
}

bool has_valid_pose(const custom_messages::Odometry& odom) {
    const auto& position = odom.pose.pose.position;
    const auto& orientation = odom.pose.pose.orientation;
    return std::isfinite(position.x) &&
           std::isfinite(position.y) &&
           std::isfinite(position.z) &&
           std::isfinite(orientation.x) &&
           std::isfinite(orientation.y) &&
           std::isfinite(orientation.z) &&
           std::isfinite(orientation.w) &&
           odom.header.stamp.toSec() > 0.0;
}

void signal_handler(int /*sig*/) {
    g_running.store(false);
}

class FrontRos2Localizer : public rclcpp::Node {
public:
    FrontRos2Localizer() : rclcpp::Node("fastlio2_ros2_localizer") {
        lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar_front");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu_front");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        frame_id_ = declare_parameter<std::string>("frame_id", "odom");
        child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
        config_path_ = declare_parameter<std::string>("config_path", "mid360.yaml");
        main_freq_ = declare_parameter<double>("main_freq", 5000.0);
        msr_freq_ = declare_parameter<double>("msr_freq", 50.0);
        odom_freq_ = declare_parameter<double>("odom_freq", 30.0);
        status_log_period_s_ = declare_parameter<double>("status_log_period_s", 5.0);

        fastlio_ = std::make_unique<FastLio>(config_path_, msr_freq_, main_freq_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 5);
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS().keep_last(200),
            std::bind(&FrontRos2Localizer::on_imu, this, std::placeholders::_1));
        lidar_sub_ = create_subscription<livox_interfaces::msg::CustomMsg>(
            lidar_topic_, rclcpp::SensorDataQoS().keep_last(5),
            std::bind(&FrontRos2Localizer::on_lidar, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "fastlio2 ros2 localizer up: lidar=%s imu=%s odom=%s frame=%s child=%s config=%s",
            lidar_topic_.c_str(), imu_topic_.c_str(), odom_topic_.c_str(),
            frame_id_.c_str(), child_frame_id_.c_str(), config_path_.c_str());
    }

    // 主循环结构照抄 ros1_localizer.cpp:195-235（spin -> process -> publish）
    void run() {
        auto self = shared_from_this();
        rclcpp::Rate idle_rate(100.0);
        double last_log_wall = 0.0;
        auto last_odom_publish = std::chrono::steady_clock::now();
        const auto odom_interval = std::chrono::duration<double>(1.0 / std::max(odom_freq_, 1.0));

        while (rclcpp::ok() && g_running.load()) {
            rclcpp::spin_some(self);
            fastlio_->process();

            const custom_messages::Odometry& odom = fastlio_->get_odometry();
            if (has_valid_pose(odom)) {
                const auto now = std::chrono::steady_clock::now();
                if (now - last_odom_publish >= odom_interval) {
                    nav_msgs::msg::Odometry ros_odom = to_ros_odometry(odom, frame_id_, child_frame_id_);
                    odom_pub_->publish(ros_odom);
                    tf_broadcaster_->sendTransform(to_odom_tf(ros_odom, frame_id_, child_frame_id_));
                    last_odom_publish = now;
                    odom_publish_count_ += 1;
                }
            } else {
                idle_rate.sleep();
                continue;
            }

            const double now_wall = now_seconds();
            if (status_log_period_s_ > 0.0 && now_wall - last_log_wall >= status_log_period_s_) {
                last_log_wall = now_wall;
                RCLCPP_INFO(get_logger(),
                    "fastlio ros2 localizer odom_published=%zu imu_messages=%zu lidar_messages=%zu frame=%s child=%s",
                    odom_publish_count_, imu_message_count_, lidar_message_count_,
                    frame_id_.c_str(), child_frame_id_.c_str());
            }
        }
    }

private:
    void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        fastlio_->feed_imu(to_fastlio_imu(msg));
        imu_message_count_ += 1;
    }

    void on_lidar(const livox_interfaces::msg::CustomMsg::ConstSharedPtr msg) {
        fastlio_->feed_lidar(to_fastlio_lidar(msg));
        lidar_message_count_ += 1;
    }

    static double now_seconds() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<FastLio> fastlio_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::string lidar_topic_;
    std::string imu_topic_;
    std::string odom_topic_;
    std::string frame_id_;
    std::string child_frame_id_;
    std::string config_path_;
    double main_freq_ = 5000.0;
    double msr_freq_ = 50.0;
    double odom_freq_ = 30.0;
    double status_log_period_s_ = 5.0;

    size_t imu_message_count_ = 0;
    size_t lidar_message_count_ = 0;
    size_t odom_publish_count_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

    try {
        auto node = std::make_shared<FrontRos2Localizer>();
        node->run();
    } catch (const std::exception& exc) {
        RCLCPP_FATAL(rclcpp::get_logger("fastlio2_ros2_localizer"),
                     "fastlio ros2 localizer failed: %s", exc.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
