#include "encoder_to_odom/encoder_converter.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace ENCODER_CONVERTER;

void Converter::run()
{
    this->processor.processData();
    this->publishOdomData();
}

void Converter::publishOdomData()
{

    nav_msgs::msg::Odometry rosOdomMsg;
    // Header
    std_msgs::msg::Header odomHeader;
    odomHeader.frame_id = "odom";
    odomHeader.stamp = this->get_clock()->now();

    rosOdomMsg.header = odomHeader;
    // Child frame
    rosOdomMsg.child_frame_id = "base_link";

    // Twist with Covariance
    geometry_msgs::msg::TwistWithCovariance odomTwistCovariance;
    geometry_msgs::msg::Twist odomTwist;

    auto velocity = this->processor.getVelocity();
    auto deltaTime = this->processor.getDeltaTime();

    geometry_msgs::msg::Vector3 linearVelocity;
    linearVelocity.x = 0.0;
    linearVelocity.y = velocity.linearY;
    linearVelocity.z = deltaTime;
    odomTwist.linear = linearVelocity;

    geometry_msgs::msg::Vector3 angularVelocity;
    angularVelocity.x = 0.0;
    angularVelocity.y = 0.0;
    angularVelocity.z = velocity.angularZ;
    odomTwist.angular = angularVelocity;
    odomTwistCovariance.twist = odomTwist;

    rosOdomMsg.twist = odomTwistCovariance;

    // Pose with Covariance
    geometry_msgs::msg::PoseWithCovariance odomPoseCovariance;
    odomPoseCovariance.covariance[0] = 1.0;
    odomPoseCovariance.covariance[7] = 1.0;
    odomPoseCovariance.covariance[14] = 1.0;
    odomPoseCovariance.covariance[21] = 1.0;
    odomPoseCovariance.covariance[28] = 1.0;
    odomPoseCovariance.covariance[35] = 1.0;

    geometry_msgs::msg::Pose odomPose;

    auto position = this->processor.getPosition();

    // TODO: clp handle the axis switch in proper ros fashion
    odomPose.position.x = -position.x;
    odomPose.position.y = position.y;
    odomPose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, position.theta);

    std::cout << "Theta (degrees): " << (position.theta) * 180 / PI << std::endl;
    // odomPose.orientation.z = position.theta;
    // odomPose.orientation.w = 1.0;
    odomPose.orientation.x = q.x();
    odomPose.orientation.y = q.y();
    odomPose.orientation.z = q.z();
    odomPose.orientation.w = q.w();

    odomPoseCovariance.pose = odomPose;

    rosOdomMsg.pose = odomPoseCovariance;

    this->odomPublisher->publish(rosOdomMsg);
}

void Converter::encoderCallback(const luci_messages::msg::LuciEncoders::SharedPtr msg)
{
    // TODO: clp - because encoder data comes in as a paired message the odom sync function is
    // likely unneeded for this node but may make sense to keep in general library
    auto leftAngle = msg->left_angle;
    auto rightAngle = msg->right_angle;
    this->processor.currentSec = msg->seconds;
    this->processor.currentNano = msg->nanoseconds;

    this->processor.updateCurrentValue(leftAngle, Motor::LEFT);
    this->processor.updateCurrentValue(rightAngle, Motor::RIGHT);
    this->run();

    // if (this->processor.getDeltaTime() > 0.01)
    // {
    this->processor.lastSec = this->processor.currentSec;
    this->processor.lastNano = this->processor.currentNano;
    // }
}
