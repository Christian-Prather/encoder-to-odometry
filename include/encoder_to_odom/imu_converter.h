#pragma once
// #include "encoder_to_odom/odometry.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include <luci_messages/msg/luci_imu.hpp>

#define PI 3.14159265

namespace IMU_CONVERTER
{

// Subscribe to IMUs
// Pack data as ROS message (IMU)
// Publish ROS message

/**
 * @brief Converter node for sending odom data over ros network
 *
 */
class Converter : public rclcpp::Node
{
  public:
    Converter() : Node("imu_odom")
    {
        /// IMU publisher, publishes on the encoder/odom topic
        imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu/odom", 1);

        /// IMU subscriber, listens for IMU messages from LUIC
        imuSubscription = this->create_subscription<luci_messages::msg::LuciImu>(
            "luci/imu", 10, std::bind(&Converter::imuCallback, this, std::placeholders::_1));

        /// Main logic of the node
        this->run();
    }

    /**
     * @brief Convert and publish odom data over the ros network
     *
     */
    void publishImuData();

  private:
    /// Encoder to odom converter
    // OdometryProcessor processor = OdometryProcessor(WHEEL_CIRCUMFRANCE, WHEEL_BASE, GEAR_RATIO);

    /// Odom publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
    /// Encoder subscriber
    rclcpp::Subscription<luci_messages::msg::LuciImu>::SharedPtr imuSubscription;

    // geometry_msgs::msg::Quaternion firstReading;
    float firstReading = 0.0;
    bool first = true;

    /**
     * @brief Encoder subscriber call back
     *
     * @param msg
     */
    void imuCallback(const luci_messages::msg::LuciImu::SharedPtr msg);

    /**
     * @brief Main logic of node
     *
     */
    void run();
};

} // namespace IMU_CONVERTER
