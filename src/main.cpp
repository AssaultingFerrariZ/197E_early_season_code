#include "RamseteController.hpp"
#include "definitions.hpp"
#include "autons.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <ios>
#include <queue>
#include "main.h"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
using namespace RTMotionProfile;
typedef std::shared_ptr<RTMotionProfile::Bezier> BezierPtr;
template<typename T>
std::string toString(T to) {
	std::stringstream ss;
	ss << to;
	return ss.str();
}


double cubeScalar(double input) {
	double cube = pow(input, 3)/100.0;
	return cube/100;
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
	ladyBrown.set_brake_mode_all(pros::v5::MotorBrake::hold);
	colorSensor.set_led_pwm(50);

	// pros::Task checkAuto([&] {
	// 	while (1) {
	// 		if (autoActive) autoHasBeenActive = true;
	// 	}
	// });

	// pros::Task screenTask([&] {
	// 	while (1) {
	// 		pros::lcd::print(3, "%.2f Heading", robot->getPose().theta);  // Prints status of the emulated screen LCDs
	// 		pros::lcd::print(1, "%.2f X", robot->getPose().x);  // Prints status of the emulated screen LCDs
	// 		pros::lcd::print(2, "%.2f Y", robot->getPose().y);  
	// 		// pros::lcd::print(3, "Proximity: %.2f", colorSensor.get_proximity());
	// 		pros::delay(20);
	// 	}
	// });

	//run the task asynchronously
	pros::Task armLiftTask([&] {
		while (1) {
			static lemlib::PID armPID(5, 0, 2, 0, true);
			double error = arm_target - ladyBrownRotation.get_angle() / 100.0;
			double output = clamp(error, -127, 127);
			if (arm_pid_enabled) ladyBrown.move(output);
			pros::delay(10);
		}
	});
	
	//run the function in a task for asynchronous detection for both auton and driver
	pros::Task colorSorter([&] {
	
		while (1) { //infinitely loop the task for the program's lifetime

			//determine if the ring is red or blue by comparing RGB values
			RingColor ringColor;
			double hue = colorSensor.get_hue();
			if (colorSensor.get_proximity() > 150) {
				if (hue < 50 && hue > 0) {
					ringColor = RED;
				} else if (hue < 240 && hue > 180) {
					ringColor = BLUE;
				} else {
					ringColor = UNKNOWN;
				}
			}
			if (intakeDistance.get_distance() <= 35) {
				if (color_sorting_enabled && redSide != ringColor && ringColor != UNKNOWN) {
					stopIntakeControl = true;
					pros::delay(160);
					intake.move(-127);
					pros::delay(75);
					if (autoActive) intake.move(127); 
					stopIntakeControl = false;
				}
			}
			
			pros::delay(10);
		}
	});	

	master.clear();
	
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
ASSET(pptest_txt)
void autonomous() {
	master.clear();
	master.print(1, 1, "Entered Auto");
	autoSelected = true;
	autoActive = true;
	// auto selection = autonSelectorMap.find(currentAutoSelection);
	// if (selection != autonSelectorMap.end()) {
	// 	selection->second.second();
	// }

	pros::lcd::print(7, "hi");
	BezierPtr testPath = std::make_shared<RTMotionProfile::Bezier>(RTMotionProfile::Bezier({-0, -0}, {-0, -0}, {-0, -0}, {-0, 24}));
	pros::lcd::print(8, "hello");
	generator->generateProfile(testPath);	
	ramsete->execute_current_profile();

	autoActive = false;
	
}

void flipMogo() {
	mogoState = !mogoState;
	mogo.set_value(mogoState);
}

void flipHang() {
	hangState = !hangState;
	hang.set_value(hangState);
	
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
	color_sorting_enabled = false;
	bool manualControl = false;
	bool arm_in_load_pos = false;
	robot->cancelAllMotions();
	//profile generation benchmarking
	// BezierPtr testPath;
	// int start_time = pros::millis();
	// testPath = std::make_shared<RTMotionProfile::Bezier>(RTMotionProfile::Bezier({-12, -36}, {-12, -60}, {-36, -36}, {-36, -60}));
	// generator->generateProfile(testPath);
	// int end_time = pros::millis();
	// std::cout << "Start Time: " << start_time << "ms, End Time: " << end_time << "ms, Total Time: " << end_time - start_time << "ms, " << std::endl;
	// std::cout << "Length: " << generator->getProfile().size() * generator->get_delta_d() << " in" << std::endl;
	ladyBrown.move(0);
	if (!autoSelected) master.print(2, 1, autonSelectorMap[currentAutoSelection].first.c_str());
	while (!autoSelected && !pros::c::competition_is_connected()) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			master.clear_line(2);
			pros::delay(50);
			currentAutoSelection = (int)clamp(currentAutoSelection-1, 1, autonSelectorMap.size());
			master.print(2, 1, autonSelectorMap[currentAutoSelection].first.c_str());
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			master.clear_line(2);
			pros::delay(50);
			currentAutoSelection = (int)clamp(currentAutoSelection+1, 1, autonSelectorMap.size());
			master.print(2, 1, autonSelectorMap[currentAutoSelection].first.c_str());
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			master.clear();
			autoSelected = true;
		}
	}
	while (true) {
		robot->arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		// leftSide.move_voltage((speedRatio(cubeScalar(
		// 	master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + 
		// 	master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)))));
		// rightSide.move_voltage((speedRatio(cubeScalar(
		// 	master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - 
		// 	master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)))));

		if (!stopIntakeControl) {
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake.move(127);
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) intake.move(-127);
			else intake.move(0);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))  {
			static int cycle = 1;
			if (cycle == 1) {
				color_sorting_enabled = false;
				cycle++;
			} else if (cycle == 2) {
				color_sorting_enabled = true;
				redSide = RED;
				cycle++;
			} else if (cycle == 3) {
				color_sorting_enabled = true;
				redSide = BLUE;
				cycle = 1;
			}
			bool isRed;
			if (redSide == RED) isRed = true;
			else if (redSide == BLUE) isRed = false;
			master.clear_line(2);
			pros::delay(50);
			master.print(2, 1, "Sort: %d, red: %d", color_sorting_enabled, isRed);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) flipMogo();
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) flipHang();

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			static bool intakeState = false;
			intakeLift.set_value(intakeState);
			intakeState = !intakeState;
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			doinkerState = !doinkerState;
			doinker.set_value(doinkerState);
		}
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && ladyBrownRotation.get_angle() / 100.0 < 215) {
			arm_pid_enabled = false;
			manualControl = true;
			ladyBrown.move(127);
		} else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			arm_pid_enabled = true;
			if (!manualControl) arm_in_load_pos = !arm_in_load_pos;
			arm_target = arm_in_load_pos ? (manualControl ? LOAD_ARM_POS - 1.5 : LOAD_ARM_POS + 1.5): BASE_ARM_POS;
			manualControl = false;
		} else if (!arm_pid_enabled) {
			ladyBrown.move(0);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			odomRetract.set_value(true);
		}

		pros::delay(20); // Run for 20 ms then update
	}
}