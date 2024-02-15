#include "encoder_to_odom/imu_converter.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace IMU_CONVERTER;

void Converter::run()
{
    // this->processor.processData();
    // this->publishOdomData();
}

void Converter::publishImuData()
{

    // sensor_msgs::msg::Imu rosImuMsg;
    // // Header
    // std_msgs::msg::Header odomHeader;
    // odomHeader.frame_id = "base_link";
    // odomHeader.stamp = this->get_clock()->now();

    // rosImuMsg.header = odomHeader;

    // geometry_msgs::msg::Quaternion orientation;
    // orientation.x = 0.0;
    // orientation.y = 0.0;
    // orientation.z = 0.0;
    // orientation.w = 1.0;

    // geometry_msgs::msg::Vector3 linearVelocity;
    // linearVelocity.x = 0.0;
    // linearVelocity.y = 0.0;
    // linearVelocity.z = 0.0;

    // geometry_msgs::msg::Vector3 angularVelocity;
    // angularVelocity.x = 0.0;
    // angularVelocity.y = 0.0;
    // angularVelocity.z = 0.0;

    // // Pose with Covariance
    // odomPoseCovariance.covariance[0] = 1.0;
    // odomPoseCovariance.covariance[7] = 1.0;
    // odomPoseCovariance.covariance[14] = 1.0;
    // odomPoseCovariance.covariance[21] = 1.0;
    // odomPoseCovariance.covariance[28] = 1.0;
    // odomPoseCovariance.covariance[35] = 1.0;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, position.theta);

    // odomPose.orientation.x = q.x();
    // odomPose.orientation.y = q.y();
    // odomPose.orientation.z = q.z();
    // odomPose.orientation.w = q.w();

    // this->odomPublisher->publish(rosImuMsg);
}

void Converter::imuCallback(const luci_messages::msg::LuciImu::SharedPtr msg)
{

    sensor_msgs::msg::Imu rosImuMsg;

    // Orientation
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = msg->quaternion_x;
    orientation.y = msg->quaternion_y;
    orientation.z = msg->quaternion_z;
    orientation.w = msg->quaternion_w;

    geometry_msgs::msg::Vector3 linearAcceleration;
    linearAcceleration.x = msg->acceleration_x;
    linearAcceleration.y = msg->acceleration_y;
    linearAcceleration.z = msg->acceleration_z;

    geometry_msgs::msg::Vector3 angularVelocity;
    angularVelocity.x = msg->gyro_x;
    angularVelocity.y = msg->gyro_y;
    angularVelocity.z = msg->gyro_z;

    rosImuMsg.header = msg->header;
    rosImuMsg.orientation = orientation;
    rosImuMsg.angular_velocity = angularVelocity;
    rosImuMsg.linear_acceleration = linearAcceleration;

    // this->run();
    this->imuPublisher->publish(rosImuMsg);
}
