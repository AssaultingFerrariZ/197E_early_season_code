#include "autons.hpp"
#include "definitions.hpp"
#include "pros/misc.hpp"
#include "main.h"
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
float wrap180(float angle) {
	while (!(angle <= 180 && angle >= -180)) {
		if (angle < -180) angle += 360;
		if (angle > 180) angle -= 360;
	}
	return angle;
}

float clamp(float output, float min, float max) {
	if (output < min) output = min;
	if (output > max) output = max;
	return output;
}

void armMacro() {
	static bool armInLoadPos = false;
	static lemlib::PID retractPID(1.1, 0.3, 0, 2, false);
	static lemlib::PID scorePID(3.2, 0, 0, 0, false);
	retractPID.reset();
	scorePID.reset();
	std::cout << "function began" << std::endl;
	double time = 0;
	while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		double error = wrap180(SCORE_ARM_POS - (ladyBrownRotation.get_angle() / 100.0));
		double output = clamp(scorePID.update(error), -127, 127);
		// std::cout << "Error: " << error <<", Angle: " << ladyBrownRotation.get_angle() / 100. << ", Output: " << output << ", Scoring Position" << std::endl;
		ladyBrown.move(output);
		// pros::lcd::print(1, "Scoring Position");
		// pros::lcd::print(2, "Error: %.2f", error);
		// pros::lcd::print(3, "Angle: %.2f", ladyBrownRotation.get_angle());
		if (fabs(error) <= 5 || time > 2500) {
			armInLoadPos = !armInLoadPos;
			// scorePID.print_integral();
			scorePID.reset();
			break;
		}

		time += 10; 
		pros::delay(10);
	}
	armInLoadPos = !armInLoadPos;
	time = 0;
	while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		double error = wrap180((armInLoadPos ? LOAD_ARM_POS : BASE_ARM_POS) - ladyBrownRotation.get_angle() / 100.0);
		double output = clamp(retractPID.update(error), -127, 127);
		ladyBrown.move(output);
		// pros::lcd::print(1, "Retracting Position");
		// pros::lcd::print(2, "Error: %.2f", error);
		// pros::lcd::print(3, "Angle: %.2f", ladyBrownRotation.get_angle());
		// std::cout << "Error: " << error <<", Angle: " << ladyBrownRotation.get_angle() / 100.0 << ", Output: " << output << ", Loading Position: " << armInLoadPos << std::endl;
		if (fabs(error) <= 0.4 || time > 3000) {
			retractPID.print_integral();
			retractPID.reset();
			break;
		}

		time += 10;
		pros::delay(10);

	}
	ladyBrown.brake();
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
	// pros::Task screenTask([&] {
	// 	while (1) {
	// 		pros::lcd::print(3, "%.2f Heading", robot->getPose().theta);  // Prints status of the emulated screen LCDs
	// 		pros::lcd::print(1, "%.2f X", robot->getPose().x);  // Prints status of the emulated screen LCDs
	// 		pros::lcd::print(2, "%.2f Y", robot->getPose().y);  // Prints status of the emulated screen LCDs
	// 		pros::delay(20);
	// 	}
	// });
	pros::Task armLiftTask([&] {
		while (1) {
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				armMacro();
				while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {}
        	}
			pros::delay(10);
		}
	});
	pros::Task colorSorter([&] {
		while (1) {
			bool ringColorRed = (colorSensor.get_rgb().red > colorSensor.get_rgb().blue);
			if (colorSensor.get_proximity() > 230 && color_sorting_enabled) {
				if (ringColorRed == redSide) continue;
				else {
					stopIntake = true;
					pros::delay(35);
					intake.move(0);
					pros::delay(750);
					stopIntake = false;
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

void competition_initialize() {
	
}

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
	auto selection = autonSelectorMap.find(currentAutoSelection);
	if (selection != autonSelectorMap.end()) {
		selection->second.second();
	}
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
	robot->cancelAllMotions();
	//profile generation benchmarking
	BezierPtr testPath;
	int start_time = pros::millis();
	testPath = std::make_shared<RTMotionProfile::Bezier>(RTMotionProfile::Bezier({-12, -36}, {-12, -60}, {-36, -36}, {-36, -60}));
	generator->generateProfile(testPath);
	int end_time = pros::millis();
	std::cout << "Start Time: " << start_time << "ms, End Time: " << end_time << "ms, Total Time: " << end_time - start_time << "ms, " << std::endl;
	std::cout << "Length: " << generator->getProfile().size() * generator->get_delta_d() << " in" << std::endl;
	
	bool autoSelected = false;
	master.print(2, 1, "Select Auton");
	while (!autoSelected && pros::competition::is_disabled()) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			currentAutoSelection = (int)clamp(currentAutoSelection-1, 1, autonSelectorMap.size());
			master.print(2, 1, autonSelectorMap[currentAutoSelection].first.c_str());
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			currentAutoSelection = (int)clamp(currentAutoSelection+1, 1, autonSelectorMap.size());
			master.print(2, 1, autonSelectorMap[currentAutoSelection].first.c_str());
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			autoSelected = true;
		}
	}
	
	while (true) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			mogo1state = !mogo1state;
			mogo2state = !mogo2state;
			mogo1.set_value(mogo1state);
			mogo2.set_value(mogo2state);
		}
		robot->arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		if (!stopIntake) {
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake.move(127);
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake.move(-127);
			else intake.move(0);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))  {
			redSide = !redSide;
		}

		pros::lcd::print(3, "Proximity %d", colorSensor.get_proximity());
		pros::lcd::print(4, toString(redSide).c_str());
		// Arcade control scheme
		// Sets right motor voltage
		pros::delay(20); // Run for 20 ms then update
	}
}