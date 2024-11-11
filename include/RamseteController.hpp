#pragma once

#include "Eigen/Dense"
#include "Eigen/src/Core/Matrix.h"
#include "MotionProfile.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp" // IWYU pragma: keep
#include <cmath>
#include <memory>
class RamseteController {
    private:
        double b;
        double zeta;
        std::shared_ptr<RTMotionProfile::ProfileGenerator> _generator;
        std::shared_ptr<lemlib::Chassis> _chassis;
    public:
        RamseteController(std::shared_ptr<lemlib::Chassis> _chassis, std::shared_ptr<RTMotionProfile::ProfileGenerator>, double b, double zeta);
        Eigen::Vector3d calculateLocalError(RTMotionProfile::Pose targetPose); 
        void execute_current_profile();
};