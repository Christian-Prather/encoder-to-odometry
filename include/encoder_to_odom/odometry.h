#pragma once
#include <iostream>
#include <math.h>

/**
 * @brief Enum to determin which motor an encoder is attached to
 *
 */
enum class Motor
{
    LEFT,
    RIGHT
};

struct Position
{
    float x;
    float y;
    float theta;
};

class OdometryProcessor
{
  public:
    /**
     * @brief Construct a new Odometry Processor object
     *
     * @param wheelCircumfrance Circumfrance in meters of robots wheels
     * @param wheelBase Distance between center of wheels in meters
     * @param gearRatio Number of encoder rotations per 1 wheel rotation, ex) an encoder mounted
     * directly on the robots wheel this number would be 1.0
     */
    OdometryProcessor(float wheelCircumfrance, float wheelBase, float gearRatio);

    /**
     * @brief Update the
     *
     * @param value
     */
    void updateCurrentValue(float value, Motor motor);

    /**
     * @brief Calculate the total distance the robot moved in the x axis
     *
     */
    void calculateDistanceMovedX();

    /**
     * @brief Calculate the total distance the robot moved in the y axis
     *
     */
    void calculateDistanceMovedY();

    /**
     * @brief Check if the robot has procuced new left and right motor readings
     *
     * @return true new values for both motor encoders are available
     * @return false new values for both motor encoders are NOT yet available
     */
    bool sync();

    /**
     * @brief Run all private calls to process new data frame
     *
     */
    void processData();

    /**
     * @brief Get the Position object
     *
     * @return Position (x,y,theta) or robot in odom coordinate frame
     */
    Position getPosition();

  private:
    /**
     * @brief Calculates the degrees the encoder moved in one frame
     *
     * @param motor Which drive motor the current angle reading should be associated with
     * @return float Degrees encoder moved in frame
     */
    float calculateDegreesTraveledInFrame(Motor motor);

    /**
     * @brief Handle the rollover / rollunder of encoders (360->1), (1->360)
     *
     * @param currentDegreeReading Current reading from the encoder
     * @param lastDegreeReading Last recorded reading from the encoder
     * @return float delta degree between last and current frame of the encoder
     */
    float handleRollover(float currentDegreeReading, float lastDegreeReading);

    /**
     * @brief Calculate the meters traveld for a motor in a given frame
     *
     * @param degreesTraveledInFrame The degrees the encoder traveled in a given frame
     * @return float The meters traveled of a wheel in a given frame
     */
    float calculateMetersTraveled(float degreesTraveledInFrame);

    /**
     * @brief Calculate the distance the robot traveled in a given frame
     *
     * @param leftDistance Distance in meters the left motor drove
     * @param rightDistance Distance in meters the right motor drove
     */
    void calculateFrameDistance(float leftDistance, float rightDistance);

    /**
     * @brief Calculate the angle change of a given frame
     *
     * @param leftDistance Distance in meters traveled by the left wheel
     * @param rightDistance Distance in meters traveled by the right wheel
     */
    void calculateHeadingAngleDelta(float leftDistance, float rightDistance);

    /// Circumfrance of robot wheels in meters
    float wheelCircumfrance = 0.0;

    /// Distance between wheel centers of robot in meters
    float wheelBase = 0.0;

    /// Number of rotations encoder makes per 1 wheel rotation
    float gearRatio = 1.0;

    /// Angle value that a delta change triggers a rollover
    float rollOverThreshold = 200.0;

    /// The last processed Delta angle between encoder frames
    float previousLeftDegree = 0.0;
    float previousRightDegree = 0.0;

    /// Sync trackers to determin if new data has been produced by the encoders
    bool leftSync = false;
    bool rightSync = false;

    /// Total degrees the left motor has traveled since being turned on
    float totalLeftDegreesTraveled = 0.0;
    /// Total degrees the right motor has traveled since being turned on
    float totalRightDegreesTraveled = 0.0;

        /// Total degrees the left motor has traveled since being turned on
    float totalLeftMeterTraveled = 0.0;
    /// Total degrees the right motor has traveled since being turned on
    float totalRightMetersTraveled = 0.0;

    /// Current angle the robot is at relative to start from being turned on
    float currentTotalPoseTheta = 0.0;

    /// Distance the robot has traveled in a given frame
    float frameDistance = 0.0;

    // TODO: clp Maybe do a struct for these positions to relate them
    /// Total distance the robot has moved in the x axis
    float totalDistanceX = 0.0;

    /// Total distance the robot has moved in the y axis
    float totalDistanceY = 0.0;

    /// Current reading from the left encoder
    float currentLeftReading = 0.0;
    /// Current reading from the right encoder
    float currentRightReading = 0.0;

    /// Number of readings to throw out before considering the system stabilized
    int stablizationAmount = 3;
};