#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/logger/message.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/util.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include "pros/optical.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "MotionProfile.hpp"
#include "RamseteController.hpp"
#include <cstddef>
#include <math.h>
#include <memory>
#include <ostream>
#include <sstream>
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
typedef std::shared_ptr<RTMotionProfile::Bezier> BezierPtr;

pros::Controller master(pros::E_CONTROLLER_MASTER);
template<typename T>
std::string toString(T to) {
	std::stringstream ss;
	ss << to;
	return ss.str();
}
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
pros::Optical colorSensor(11);
pros::Motor intake(19, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup ladyBrown({21}, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg); 
pros::Rotation ladyBrownRotation(2);
pros::adi::DigitalOut mogo1('A');
bool mogo1state = LOW;
pros::adi::DigitalOut mogo2('B');
bool mogo2state = LOW;
pros::MotorGroup leftSide({1, 12, 13}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup rightSide({4, 7, 9}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
lemlib::Drivetrain drivetrain(&leftSide, &rightSide, -11.5, 4.125, 200, 8);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              1, // derivative gain (kD)
                                              3, // anti windup
                                              2, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
pros::Rotation horizontal_sensor(4);
pros::Rotation vertical_sensor(4);
pros::Imu imu(10);
// vertical tracking wheel encoder
// horizontal tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_2, -2.5);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, -5.5);
// vertical tracking wheel
lemlib::OdomSensors odom(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

std::shared_ptr<lemlib::Chassis> robot(new lemlib::Chassis(drivetrain, lateral_controller, angular_controller, odom));
// lemlib::Chassis robot=(lemlib::Chassis(drivetrain, lateral_controller, angular_controller, odom));

const double MAX_SPEED = 450 * (3.25*M_PI) / 60;
RTMotionProfile::Constraints mp_constraints(
	MAX_SPEED,
	MAX_SPEED*3, 
	0.1, 
	MAX_SPEED*3, 
	MAX_SPEED*100, 
	12);
std::shared_ptr<RTMotionProfile::ProfileGenerator> generator(new RTMotionProfile::ProfileGenerator(
	std::make_shared<RTMotionProfile::Constraints>(mp_constraints),
	0.1
));
// std::shared_ptr<RamseteController> ramsete(new RamseteController(
// 	robot, 
// 	generator,
// 	2.0,
// 	0.7
// ));

const double BASE_ARM_POS = 60, LOAD_ARM_POS = 5, SCORE_ARM_POS = 120;

void fakeTask() {
	std::cout << "Task Ran" << std::endl;
}

void armMacro() {
	static bool armInLoadPos = false;
	std::cout << "Task Entered" << std::endl;
	lemlib::PID armPID(0.1, 0, 0, 3, false);
	while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		double error = SCORE_ARM_POS - ladyBrownRotation.get_angle();
		double output = armPID.update(error);
		// ladyBrown.move(output);
		// pros::lcd::print(1, "Scoring Position");
		// pros::lcd::print(2, "Error: %.2f", error);
		// pros::lcd::print(3, "Angle: %.2f", ladyBrownRotation.get_angle());
		if (error <= 1) {
			armInLoadPos = !armInLoadPos;
			break;
		}
		pros::delay(10);
	}
	pros::lcd::clear();
	armInLoadPos = !armInLoadPos;
	while (1) {
		double error = (armInLoadPos ? LOAD_ARM_POS : BASE_ARM_POS) - ladyBrownRotation.get_angle();
		double output = armPID.update(error);
		// ladyBrown.move(output);
		// pros::lcd::print(1, "Retracting Position");
		// pros::lcd::print(2, "Error: %.2f", error);
		// pros::lcd::print(3, "Angle: %.2f", ladyBrownRotation.get_angle());
		// pros::lcd::print(4, (armInLoadPos ? "retracting to loading position" : "retracting to non-loading position"));
		if (error <= 1) break;
		pros::delay(10);

	}
}

/**
 * Runs initialization code. This occuras soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	robot->calibrate();
	pros::lcd::initialize();
	// pros::Task screenTask([&] {
	// 	while (1) {
	// 		pros::lcd::print(3, "%.2f Heading", robot->getPose().theta);  // Prints status of the emulated screen LCDs
	// 		pros::lcd::print(1, "%.2f X", robot->getPose().x);  // Prints status of the emulated screen LCDs
	// 		pros::lcd::print(2, "%.2f Y", robot->getPose().y);  // Prints status of the emulated screen LCDs
	// 		pros::delay(20);
	// 	}
	// });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.""
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own tas
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

float speedRatio(double percent) {
	return percent * (127.0/100);
}
void autonomous() {
	// master.clear();
	robot->setPose(0, 0, 0);
	robot->moveToPose(12, 24, 0, 5000, {
		.minSpeed = (127.0/2),
		.earlyExitRange = 1
	});
	robot->moveToPose(0, 36, 0, 5000, {
		.minSpeed = speedRatio(75),
		.earlyExitRange = 5
	});
	robot->moveToPose(-12, 24, 90, 5000, {
		.forwards = false, 
		// .lead = 0.4,
		// .minSpeed = speedRatio(35), 
		// .earlyExitRange = 1,
	});
	robot->moveToPose(0, 0, 0, 5000, {.forwards = false});
	// robot->moveToPose(15, 15, 0, 5000, {.minSpeed = 72, .earlyExitRange = 8});
	// robot->moveToPose(30, 0, 0, 5000);
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	static std::shared_ptr<pros::Task> armLiftTask = nullptr;  // Static pointer to track the armLift task
	robot->cancelAllMotions();
	//profile generation benchmarking
	BezierPtr testPath;
	int start_time = pros::millis();
	testPath = std::make_shared<RTMotionProfile::Bezier>(RTMotionProfile::Bezier({-12, -36}, {-12, -60}, {-36, -36}, {-36, -60}));
	generator->generateProfile(testPath);
	int end_time = pros::millis();
	std::cout << "Start Time: " << start_time << "ms, End Time: " << end_time << "ms, Total Time: " << end_time - start_time << "ms, " << std::endl;
	std::cout << "Length: " << generator->getProfile().size() * generator->get_delta_d() << " in" << std::endl;
	while (true) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			mogo1state = !mogo1state;
			mogo2state = !mogo2state;
			mogo1.set_value(mogo1state);
			mogo2.set_value(mogo2state);
		}
		robot->arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake.move(127);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake.move(-127);
		else intake.move(0);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (armLiftTask != nullptr) {
                if (armLiftTask->get_state() == pros::E_TASK_STATE_RUNNING) {
                    std::cout << "Previous task still running" << std::endl;
                    continue;
                }
                
                std::cout << "Removing previous task" << std::endl;
                armLiftTask->remove();  // Remove previous task if it exists
                armLiftTask = nullptr;  // Reset the task pointer after removal
            }

            std::cout << "Starting new task" << std::endl;
            armLiftTask = std::make_shared<pros::Task>(armMacro);  // Launch the armMacro task
        }

		// Arcade control scheme
		// Sets right motor voltage
		pros::delay(20); // Run for 20 ms then update
	}
}