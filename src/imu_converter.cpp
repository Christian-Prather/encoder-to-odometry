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

    if (this->first)
    {

        // this->firstReading.x = msg->quaternion_x;
        // this->firstReading.y = msg->quaternion_y;
        // this->firstReading.z = msg->quaternion_z;
        // this->firstReading.w = msg->quaternion_w;
        this->first = false;
        this->firstReading = -msg->euler_z;
        std::cout << "Starting angle " << this->firstReading << std::endl;
        return;
    }

    // Orientation
    geometry_msgs::msg::Quaternion orientation;
    tf2::Quaternion q;

    float currentTotalPoseTheta = -msg->euler_z - this->firstReading; // Degrees
    currentTotalPoseTheta *= (PI / 180);
    if (currentTotalPoseTheta > PI)
    {
        currentTotalPoseTheta -= 2.0 * PI;
    }
    else if (currentTotalPoseTheta < -PI)
    {
        currentTotalPoseTheta += 2.0 * PI;
    }

    std::cout << "Theta(Degrees): " << -msg->euler_z - this->firstReading
              << " rollover: " << currentTotalPoseTheta * (180 / PI) << std::endl;

    q.setRPY(0, 0, currentTotalPoseTheta);

    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    rosImuMsg.orientation_covariance[0] = 1.0;
    rosImuMsg.orientation_covariance[7] = 1.0;
    rosImuMsg.orientation_covariance[14] = 1.0;
    rosImuMsg.orientation_covariance[21] = 1.0;
    rosImuMsg.orientation_covariance[28] = 1.0;
    rosImuMsg.orientation_covariance[35] = 1.0;

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
