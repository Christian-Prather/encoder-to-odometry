#pragma once
#include "encoder_to_odom/odometry.h"
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include <luci_messages/msg/luci_encoders.hpp>

// TODO: clp - make config arguments
constexpr float WHEEL_CIRCUMFRANCE = 1.0373;
constexpr float WHEEL_BASE = 0.4926;  // 0.56
constexpr float GEAR_RATIO = 2.38462; // 500 prototype -> 2.1138 (761 turns per wheel revolution)

namespace ENCODER_CONVERTER
{

// Subscribe to encoders
// Process them with OdometryProcessor
// Pack data as ROS message
// Publish ROS message

/**
 * @brief Converter node for sending odom data over ros network
 *
 */
class Converter : public rclcpp::Node
{
  public:
    Converter() : Node("encoder_odom")
    {
        /// Odom publisher, publishes on the encoder/odom topic
        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("encoder/odom", 1);

        /// Encoder subscriber, listens for encoder messages from LUIC
        encoderSubscription = this->create_subscription<luci_messages::msg::LuciEncoders>(
            "luci/encoders", 10,
            std::bind(&Converter::encoderCallback, this, std::placeholders::_1));

        /// Main logic of the node
        this->run();
    }

    /**
     * @brief Convert and publish odom data over the ros network
     *
     */
    void publishOdomData();

  private:
    /// Encoder to odom converter
    OdometryProcessor processor = OdometryProcessor(WHEEL_CIRCUMFRANCE, WHEEL_BASE, GEAR_RATIO);

    /// Odom publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    /// Encoder subscriber
    rclcpp::Subscription<luci_messages::msg::LuciEncoders>::SharedPtr encoderSubscription;

    /**
     * @brief Encoder subscriber call back
     *
     * @param msg
     */
    void encoderCallback(const luci_messages::msg::LuciEncoders::SharedPtr msg);

    /**
     * @brief Main logic of node
     *
     */
    void run();
};

} // namespace ENCODER_CONVERTER
