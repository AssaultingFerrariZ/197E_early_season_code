#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
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
#include "pros/rtos.hpp"
#include <sstream>
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
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
pros::adi::DigitalOut mogo1('A');
bool mogo1state = LOW;
pros::adi::DigitalOut mogo2('B');
bool mogo2state = LOW;
pros::MotorGroup leftSide({10}, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup rightSide({-1}, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::deg);
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
pros::Rotation horizontal_encoder(4);
pros::Imu inertial(2);
// vertical tracking wheel encoder
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -10.5);
// vertical tracking wheel
lemlib::OdomSensors odom(nullptr, nullptr, nullptr, nullptr, &inertial);

lemlib::Chassis robot(drivetrain, lateral_controller, angular_controller, odom);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	robot.calibrate();
	pros::lcd::initialize();
	pros::Task screenTask([&] {
		while (1) {
			pros::lcd::print(3, "%.2f Heading", robot.getPose().theta);  // Prints status of the emulated screen LCDs
			pros::lcd::print(1, "%.2f X", robot.getPose().x);  // Prints status of the emulated screen LCDs
			pros::lcd::print(2, "%.2f Y", robot.getPose().y);  // Prints status of the emulated screen LCDs
			pros::delay(20);
		}
	});
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
	robot.setPose(0, 0, 0);
	robot.moveToPose(12, 24, 0, 5000, {
		.minSpeed = (127.0/2),
		.earlyExitRange = 1
	});
	robot.moveToPose(0, 36, 0, 5000, {
		.minSpeed = speedRatio(75),
		.earlyExitRange = 5
	});
	robot.moveToPose(-12, 24, 90, 5000, {
		.forwards = false, 
		// .lead = 0.4,
		// .minSpeed = speedRatio(35), 
		// .earlyExitRange = 1,
	});
	robot.moveToPose(0, 0, 0, 5000, {.forwards = false});
	// robot.moveToPose(15, 15, 0, 5000, {.minSpeed = 72, .earlyExitRange = 8});
	// robot.moveToPose(30, 0, 0, 5000);
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
	robot.cancelAllMotions();
	while (true) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			mogo1state = !mogo1state;
			mogo2state = !mogo2state;
			mogo1.set_value(mogo1state);
			mogo2.set_value(mogo2state);
		}
		robot.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		
		// Arcade control scheme
		// Sets right motor voltage
		pros::delay(20); // Run for 20 ms then update
	}
}