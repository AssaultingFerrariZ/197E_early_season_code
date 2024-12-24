#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "autons.hpp"
#include "MotionProfile.hpp"
#include "RamseteController.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

extern pros::Optical colorSensor;

extern pros::Controller master;

extern pros::Motor intake;
extern pros::MotorGroup ladyBrown;
extern pros::Rotation ladyBrownRotation;

extern pros::adi::DigitalOut mogo;
extern bool mogoState;

extern bool autoSelected;

extern pros::adi::DigitalOut hang;
extern bool hangState;

extern pros::adi::DigitalOut intakeLift;

extern pros::adi::DigitalOut doinker;
extern bool doinkerState;

extern pros::MotorGroup leftSide;
extern pros::MotorGroup rightSide;
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern pros::Rotation horizontal_sensor;
extern pros::Rotation vertical_sensor;
extern pros::Imu imu;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::OdomSensors odom;
extern std::shared_ptr<lemlib::Chassis> robot;
extern const double MAX_SPEED;
extern RTMotionProfile::Constraints mp_constraints;
extern std::shared_ptr<RTMotionProfile::ProfileGenerator> generator;
extern std::shared_ptr<RamseteController> ramsete;
extern bool stopIntakeControl;
extern bool redSide;
extern bool color_sorting_enabled;

void scoreArm();
void retractArm();

extern std::map<int, std::pair<std::string, std::function<void()>>> autonSelectorMap;

extern const double BASE_ARM_POS;
extern const double LOAD_ARM_POS;
extern const double SCORE_ARM_POS;

extern bool arm_in_load_pos;

extern int currentAutoSelection;