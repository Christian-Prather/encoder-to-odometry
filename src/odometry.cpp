#include "encoder_to_odom/odometry.h"

OdometryProcessor::OdometryProcessor(float wheelCircumfrance, float wheelBase, float gearRatio)
    : wheelCircumfrance(wheelCircumfrance), wheelBase(wheelBase), gearRatio(gearRatio)
{
    this->linearStartTime = std::chrono::system_clock::now();
    this->angularStartTime = std::chrono::system_clock::now();
}

float OdometryProcessor::handleRollover(float currentDegreeReading, float lastDegreeReading)
{
    // float deltaDegrees = 0.0;
    // if (currentDegreeReading != lastDegreeReading)
    // {
    float currentPreviousDelta = currentDegreeReading - lastDegreeReading;
    // float previousCurrentDelta = lastDegreeReading - currentDegreeReading;

    if (currentPreviousDelta > this->rollOverThreshold)
    {
        currentPreviousDelta = 0 - (360.0 - currentPreviousDelta);
    }

    else if (currentPreviousDelta < -this->rollOverThreshold)
    {
        currentPreviousDelta = 360.0 + currentPreviousDelta;
    }

    return currentPreviousDelta;
    // }
    // If current is same as last, return no delta
    // return 0;
}

// TODO: clp this is very  not dry one might even say this code is wet
float OdometryProcessor::calculateDegreesTraveledInFrame(Motor motor)
{

    float currentDegreeReading = 0.0;
    float lastDegreeReading = 0.0;

    if (motor == Motor::LEFT)
    {
        currentDegreeReading = this->currentLeftReading;
        lastDegreeReading = this->previousLeftDegree;
        // Update the previous reading with current for next frame
        this->previousLeftDegree = currentDegreeReading;

        // std::cout << "LEFT Current: " << currentDegreeReading << " Last: " << lastDegreeReading
        //           << std::endl;
    }
    else if (motor == Motor::RIGHT)
    {
        currentDegreeReading = this->currentRightReading;
        lastDegreeReading = this->previousRightDegree;
        // Update the previous reading with current for next frame
        this->previousRightDegree = currentDegreeReading;

        // std::cout << "RIGHT Current: " << currentDegreeReading << " Last: " << lastDegreeReading
        //           << std::endl;
    }

    float deltaDegrees = handleRollover(currentDegreeReading, lastDegreeReading);

    // Early return for n frames until system has stabilized
    if (this->stablizationAmount > 0)
    {
        this->stablizationAmount--;
        return 0.0;
    }

    if (motor == Motor::LEFT)
    {
        totalLeftDegreesTraveled += deltaDegrees;
    }
    else if (motor == Motor::RIGHT)
    {
        totalRightDegreesTraveled +=
            deltaDegrees; // TODO: clp negative because 500 install has right encoder forward as -
    }

    std::cout << "Total Left Degrees: " << totalLeftDegreesTraveled
              << " Total Right Degrees: " << totalRightDegreesTraveled << std::endl;

    return deltaDegrees;
}

// Per frame
float OdometryProcessor::calculateMetersTraveled(float totalDegrees) // Per frame
{
    float encoderRotations = totalDegrees / 360.0; // TODO: clp Apply gear ratio
    float rotations =
        encoderRotations / this->gearRatio; // TODO: clp config and de magic this (Q300/Q500)
                                            // handles gear ration of new encoders
    float metersTraveled = rotations * this->wheelCircumfrance;
    // std::cout << "Encode Rotations this Frame: " << encoderRotations
    //           << "Wheel Rotations: " << rotations << " Meter Traveled: " << metersTraveled
    //           << std::endl;
    return metersTraveled;
}

void OdometryProcessor::calculateFrameDistance(float leftDistance, float rightDistance)
{
    // auto end = std::chrono::system_clock::now();

    this->frameDistance = (rightDistance + leftDistance) / 2.0;

    this->totalDistance += this->frameDistance;

    // float deltaTime =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(end -
    //     this->linearStartTime).count() / 1000.0f;

    // int deltaSeconds = this->currentSec - this->lastSec;
    // int deltaNano = 0;

    // This assumes the delta time is never greater than 1 second
    // if (deltaSeconds > 0)
    // {
    //     // Seconds dont match we went up a second so nano delta is remainder of old plus new
    //     deltaNano += (1000000000 - this->lastNano); // Nano seconds remaing from last time
    //     deltaNano += this->currentNano;
    // }
    // else
    // {

    // deltaNano = this->currentNano - this->lastNano;
    // }

    // std::cout << "DS: " << deltaSeconds << " DN: " << deltaNano << std::endl;

    // float millisecondDelta = (deltaNano / 1000000) + (deltaSeconds * 1000);

    // float deltaTime = millisecondDelta / 1000.0f;
    // this->deltaTimeCapture = deltaTime;

    // if (this->stablizationAmount > 0)
    // {
    //     // this->stablizationAmount--;
    //     // return 0.0;
    //     return;
    // }

    // if (deltaTime < 0.01)
    // {
    //     deltaTime = 0.015;
    // }

    // M/S velocity relative to the robot frame??
    // TODO: clp make sure this ^ is true

    // if (deltaTime > 0.005)
    // {
    // float velocity = this->frameDistance / (deltaTime); // should be deltaTime
    // if (std::abs(velocity) < 5.0)
    // {
    //     if (std::abs(velocity) > std::abs(this->maxVelocity))
    //     {
    //         std::cout << "---------------------------------------------------------------------"
    //                      "----------------------------------"
    //                   << std::endl;
    //         this->maxVelocity = velocity;
    //     }

    //     std::cout << "DT: " << deltaTime << " distance: " << this->frameDistance
    //               << " M/S: " << velocity << " MAX: " << this->maxVelocity
    //               << " Total Dist: " << this->totalDistance << std::endl;
    //     this->linearVelocity = velocity;
    // }
    // }
    // this->start = std::chrono::system_clock::now();
    // this->linearStartTime = std::chrono::system_clock::now();
}

// Radians
void OdometryProcessor::calculateHeadingAngleDelta(float leftDistance, float rightDistance)
{
    // auto end = std::chrono::system_clock::now();

    // float deltaTime =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(end - this->angularStartTime)
    //         .count() /
    //     1000.0f;

    // int deltaSeconds = this->currentSec - this->lastSec;
    // int deltaNano = 0;
    // This assumes the delta time is never greater than 1 second
    // if (deltaSeconds > 0)
    // {
    //     // Seconds dont match we went up a second so nano delta is remainder of old plus new
    //     deltaNano += (1000000000 - this->lastNano); // Nano seconds remaing from last time
    //     deltaNano += this->currentNano;
    // }
    // else
    // {
    // deltaNano = this->currentNano - this->lastNano;
    // }

    // float millisecondDelta = (deltaNano / 1000000) + (deltaSeconds * 1000);

    // float deltaTime = millisecondDelta / 1000.0f;

    float difference = rightDistance - leftDistance;
    float angle = asinf(difference / this->wheelBase); // Radians

    // if (deltaTime < 0.01)
    // {
    //     deltaTime = 0.015;
    // }

    // float velocity = angle / (deltaTime);

    // this->angularVelocity = velocity;

    // std::cout << "Angular M/S: " << this->angularVelocity << std::endl;
    // if (!firstRun)
    // {
    this->currentTotalPoseTheta += (angle);
    if (this->currentTotalPoseTheta > PI)
    {
        this->currentTotalPoseTheta -= 2.0 * PI;
    }
    else if (this->currentTotalPoseTheta < -PI)
    {
        this->currentTotalPoseTheta += 2.0 * PI;
    }
    // }
    // return angle;

    // std::cout << "ld: " << leftDistance << " rd: " << rightDistance << " diff: " << difference
    //   << std::endl;
}

void OdometryProcessor::updateCurrentValue(float value, Motor motor)
{

    // if (value > 360.0)
    // {
    //     value = 360.0;
    // }
    if (motor == Motor::LEFT)
    {
        this->currentLeftReading = value;
        this->leftSync = true;
    }
    else if (motor == Motor::RIGHT)
    {
        this->currentRightReading = value;
        this->rightSync = true;
    }
}

bool OdometryProcessor::sync()
{
    if (this->leftSync && this->rightSync)
    {
        this->leftSync = false;
        this->rightSync = false;
        return true;
    }
    return false;
}

void OdometryProcessor::calculateDistanceMovedX()
{
    // if (!firstRun)
    // {

    float distanceMoved = cosf(this->currentTotalPoseTheta) * this->frameDistance;

    this->totalDistanceX += distanceMoved;
    // }
}

void OdometryProcessor::calculateDistanceMovedY()
{
    // if (!firstRun)

    // {
    float distanceMoved = sinf(this->currentTotalPoseTheta) * this->frameDistance;
    this->totalDistanceY += distanceMoved;

    // std::cout << "Frame Dist: " << this->frameDistance
    //           << " total Pose: " << this->currentTotalPoseTheta
    //           << "Distance Moved: " << distanceMoved << "totalDistY: " << totalDistanceY
    //           << std::endl;
    // }
    // this->firstRun = false;
}

Position OdometryProcessor::getPosition()
{
    Position position = {this->totalDistanceX, this->totalDistanceY, this->currentTotalPoseTheta};
    return position;
}

Velocity OdometryProcessor::getVelocity()
{
    Velocity velocity = {this->linearVelocity, this->angularVelocity};
    return velocity;
}

float OdometryProcessor::getDeltaTime() { return this->deltaTimeCapture; }

void OdometryProcessor::processData()
{
    // Make all internal calls
    if (sync())
    {

        std::cout << std::endl;
        // std::cout << "Synced" << std::endl;
        // Get distance traveled for each wheel
        // Per frame
        float currentLeftDegreesTraveled = -calculateDegreesTraveledInFrame(Motor::LEFT);
        float currentRightDegreesTraveled = calculateDegreesTraveledInFrame(Motor::RIGHT);

        // Per Frame
        // std::cout << "Left ";
        float leftMetersTraveled = calculateMetersTraveled(currentLeftDegreesTraveled);

        // std::cout << "Right ";
        float rightMetersTraveled = calculateMetersTraveled(currentRightDegreesTraveled);

        this->totalLeftMeterTraveled += leftMetersTraveled;
        this->totalRightMetersTraveled += rightMetersTraveled;

        std::cout << "LM: " << this->totalLeftMeterTraveled
                  << " RM: " << this->totalRightMetersTraveled << std::endl;

        calculateFrameDistance(leftMetersTraveled, rightMetersTraveled);

        // this->linearStartTime = std::chrono::system_clock::now();
        // this->angularStartTime = std::chrono::system_clock::now();

        calculateHeadingAngleDelta(leftMetersTraveled, rightMetersTraveled);

        calculateDistanceMovedX();
        calculateDistanceMovedY();
    }
}
