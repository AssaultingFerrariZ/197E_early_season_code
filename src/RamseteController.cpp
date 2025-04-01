#include "RamseteController.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <math.h>
#include <memory>

RamseteController::RamseteController(std::shared_ptr<lemlib::Chassis> _chassis, std::shared_ptr<RTMotionProfile::ProfileGenerator> _generator, double b, double zeta) {
        this->b = b;
        this->zeta = zeta;
        this->_chassis = _chassis;
        this->_generator = _generator;
}

Eigen::Vector3d RamseteController::calculateLocalError(RTMotionProfile::Pose targetPose){
    double current_x = _chassis->getPose().x;
    double current_y = _chassis->getPose().y;
    double current_theta = _chassis->getPose().theta;
    Eigen::Vector3d globalError;
    globalError << 
        (targetPose.x - current_x), 
        (targetPose.y - current_y), 
        (targetPose.theta-current_theta);
    Eigen::Matrix3d transformationMatrix; 
    transformationMatrix << cos(current_theta), sin(current_theta), 0.0,
                            -1*sin(current_theta), cos(current_theta), 0.0,
                            0.0, 0.0, 1.0;
    Eigen::Vector3d localError = transformationMatrix * globalError;
    return localError;
}

void RamseteController::execute_current_profile() {
    auto currentProfile = _generator->getProfile();
    double totalDistance = currentProfile.back().dist;  // Total distance in the profile
    double delta_d = _generator->get_delta_d();  // The step size for each iteration
    
    double currentDistance = 0.0;
    pros::lcd::print(1, "entered");
    int count = 0;
    while (currentDistance <= totalDistance) {
        pros::lcd::print(2, "loop ran %d times", count);
        count++;
        // Get the profile point at the current distance
        auto wheelSpeeds = _generator->getProfilePoint(currentDistance);

        // Get the local error between the robot's current pose and the target pose
        auto localError = calculateLocalError(wheelSpeeds.pose);

        // Calculate the "gain" value (control law term)
        double k = 2 * zeta * sqrt((wheelSpeeds.omega * wheelSpeeds.omega) + b * (wheelSpeeds.vel * wheelSpeeds.vel));

        // Apply the Ramsete control law to get the linear and angular velocities
        double linearVelocity = wheelSpeeds.vel * cos(localError.z()) + k * localError.x();
        double angularVelocity = wheelSpeeds.omega + k * localError.z() + ((b * wheelSpeeds.vel * sin(localError.z()) * localError.y()) / localError.z());

        // Convert linear and angular velocities to wheel velocities
        double linearWheelVelocity = linearVelocity / (3.25 * M_PI);  // Assuming the wheel diameter is 3.25 inches
        double angularWheelVelocity = angularVelocity / (3.25 * M_PI); // Same assumption as above

        // Command the motors to move with the computed velocities
        _chassis->get_drivetrain().leftMotors->move(linearWheelVelocity - angularWheelVelocity);
        _chassis->get_drivetrain().rightMotors->move(linearWheelVelocity + angularWheelVelocity);

        // Increment the distance based on the current profile point velocity and delta time
        currentDistance += delta_d;

        // Add a small delay to ensure the loop runs at an appropriate frequency
        pros::delay((delta_d/wheelSpeeds.vel)*1000);  // Delay for 20 milliseconds (adjust as necessary)
    }

    // Stop the motors when the profile is complete
    _chassis->get_drivetrain().leftMotors->move(0);
    _chassis->get_drivetrain().rightMotors->move(0);

    // Optionally, display completion message
    pros::lcd::print(3, "Completed");
}
