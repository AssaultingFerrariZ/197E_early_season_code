#pragma once

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
    double totalDistance = currentProfile.back().dist;
    double delta_d = _generator->get_delta_d();
    for (double d = 0; d <= totalDistance; d+=delta_d) {
        auto wheelSpeeds = _generator->getProfilePoint(d);
        auto localError = calculateLocalError(wheelSpeeds.pose);

        //create gain value
        double k = 2*zeta*sqrt((wheelSpeeds.omega*wheelSpeeds.omega)+b*(wheelSpeeds.vel*wheelSpeeds.vel));
        
        double linearVelocity = wheelSpeeds.vel*cos(localError.z())+k*localError.x();
        double angularVelocity = wheelSpeeds.omega+k*localError.z()+((b*wheelSpeeds.vel
            *sin(localError.z())*localError.y())/localError.z());
        
        double linearWheelVelocity = linearVelocity/(3.25*M_PI);
        double angularWheelVelocity = angularVelocity/(3.25*M_PI);

        pros::lcd::print(4, "%f\n %f", linearWheelVelocity-angularWheelVelocity, linearWheelVelocity+angularWheelVelocity);
        pros::lcd::print(6, "%f", _generator->getProfilePoint(d).vel);

        _chassis->get_drivetrain().leftMotors->move(linearWheelVelocity-angularWheelVelocity);
        _chassis->get_drivetrain().rightMotors->move(linearWheelVelocity+angularWheelVelocity);
        pros::delay(_generator->get_delta_d());
    }  
    _chassis->get_drivetrain().leftMotors->move(0);
    _chassis->get_drivetrain().rightMotors->move(0);
}