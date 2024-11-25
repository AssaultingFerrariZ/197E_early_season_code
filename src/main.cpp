#include "definitions.hpp"
#include "autons.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "main.h"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
 pros::Controller master(pros::E_CONTROLLER_MASTER);

typedef std::shared_ptr<RTMotionProfile::Bezier> BezierPtr;
template<typename T>
std::string toString(T to) {
	std::stringstream ss;
	ss << to;
	return ss.str();
}





/**
 * Runs initialization code. This occuras soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void scoreArm() {
	//define constants (kP higher than retraction to create faster scoring motion)
	static lemlib::PID scorePID(3.2, 0, 0, 0, false);
	scorePID.reset();
	double time = 0;
	//if the button is being held and killswitch is not triggered
	while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && 
			!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		//wrap the error from -180 to 180 to enable forward and reverse movement if the arm 
			//passes the target
		double error = wrap180(SCORE_ARM_POS - (ladyBrownRotation.get_angle() / 100.0));
		double output = clamp(scorePID.update(error), -127, 127);
		ladyBrown.move(output);

		//higher error tolerance because position is less precise
		if (fabs(error) <= 5 || time > 2500) {
			//since the macro automatically flips the position when function ends, flip it back to keep 
				//initial position
			arm_in_load_pos = !arm_in_load_pos;  
			scorePID.reset();
			break;
		}

		time += 10; //prevent infinite timeouts
		pros::delay(10);
	}
}

void retractArm() {
	//define PID constants
	static lemlib::PID retractPID(1.1, 0.3, 0, 2, false);
	//reset integral upon execution
	retractPID.reset();
	double time = 0;
	//killswitch to block arm from getting stuck
	while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		//target varies based on the arm's current position
		double error = wrap180(
			(arm_in_load_pos ? LOAD_ARM_POS : BASE_ARM_POS) - 
					ladyBrownRotation.get_angle() / 100.0); 
			//centidegrees to degrees
		double output = clamp(retractPID.update(error), -127, 127);
		ladyBrown.move(output);
		
		//exit the PID once within angle tolerance or timeout is reached
		if (fabs(error) <= 0.4 || time > 3000) { 
			break; 
		}

		time += 10; 
		pros::delay(10);

	}
	
}
void initialize() {
	robot->calibrate();
	pros::lcd::initialize();
	ladyBrown.set_brake_mode_all(pros::v5::MotorBrake::hold); 
	colorSensor.set_led_pwm(100);
	pros::Task screenTask([&] {
		while (1) {
			pros::lcd::print(3, "%.2f Heading", robot->getPose().theta);  // Prints status of the emulated screen LCDs
			pros::lcd::print(1, "%.2f X", robot->getPose().x);  // Prints status of the emulated screen LCDs
			pros::lcd::print(2, "%.2f Y", robot->getPose().y);  // Prints status of the emulated screen LCDs
			pros::delay(20);
		}
	});

	//run the task asynchronously
	pros::Task armLiftTask([&] {
		while (1) {
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				scoreArm();
				while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {}
				//switch arm position
				arm_in_load_pos = !arm_in_load_pos;
				retractArm();
				ladyBrown.brake();
				//block additional calls of the function while the button is being pressed
				while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {}
        	}
			pros::delay(10);
		}
	});
	
	//run the function in a task for asynchronous detection for both auton and driver
	pros::Task colorSorter([&] {
		while (1) { //infinitely loop the task for the program's lifetime

			//determine if the ring is red or blue by comparing RGB values
			bool ringColorRed = (colorSensor.get_rgb().red > colorSensor.get_rgb().blue);

			//check if both color sorting is enabled and that the ring is the object being sensed
			if (colorSensor.get_proximity() > 230 && color_sorting_enabled) {
				//if the ring color is the same as the match color, do nothing 
				if (ringColorRed == redSide) continue; 
				else {
					stopIntakeControl = true; //prevent usercontrol code from running intake
					pros::delay(50); //wait until the intake gets to the right position
					intake.move(0); //stop the intake
					// intake.move_relative(90, 127);
					pros::delay(500); //keep the intake stopped to let the ring fly off
					stopIntakeControl = false; //return intake control to usercontrol
					intake.move(127); //restart intake (for autonomous)
				}
			}
			pros::delay(10);
		}
	});	

	// pros::Task setPistons([&] {
	// 	mogo1.set_value(mogoState);
	// 	mogo2.set_value(mogoState);
	// 	hang1.set_value(hangState);
	// 	hang2.set_value(hangState);
	// 	pros::delay(20);
	// });

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
void autonomous() {
	autoSelected = true;
	// BezierPtr path(new RTMotionProfile::Bezier({0, 0}, {0.863, 15.186}, {17.254, -16.103}, {24, 0}));
	// BezierPtr straightPath(new RTMotionProfile::Bezier(
	// 	{0, 0}, 
	// 	{0, 0}, 
	// 	{0, 0}, 
	// 	{0, 24}
	// ));
	// generator->generateProfile(straightPath);
	// ramsete	->execute_current_profile();
	auto selection = autonSelectorMap.find(currentAutoSelection);
	if (selection != autonSelectorMap.end()) {
		selection->second.second();
	}
}

void flipMogo() {
	mogoState = !mogoState;
	mogo1.set_value(mogoState);
	mogo2.set_value(mogoState);
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
	robot->cancelAllMotions();
	//profile generation benchmarking
	BezierPtr testPath;
	int start_time = pros::millis();
	testPath = std::make_shared<RTMotionProfile::Bezier>(RTMotionProfile::Bezier({-12, -36}, {-12, -60}, {-36, -36}, {-36, -60}));
	generator->generateProfile(testPath);
	int end_time = pros::millis();
	std::cout << "Start Time: " << start_time << "ms, End Time: " << end_time << "ms, Total Time: " << end_time - start_time << "ms, " << std::endl;
	std::cout << "Length: " << generator->getProfile().size() * generator->get_delta_d() << " in" << std::endl;
	if (!autoSelected) master.print(2, 1, autonSelectorMap[currentAutoSelection].first.c_str());
	while (!autoSelected) {
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
		if (!stopIntakeControl) {
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake.move(127);
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake.move(-127);
			else intake.move(0);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))  {
			static int cycle = 1;
			if (cycle == 1) {
				color_sorting_enabled = false;
				cycle++;
			} else if (cycle == 2) {
				color_sorting_enabled = true;
				redSide = true;
				cycle++;
			} else if (cycle == 3) {
				color_sorting_enabled = true;
				redSide = false;
				cycle = 1;
			}
			master.clear_line(2);
			pros::delay(50);
			master.print(2, 1, "Sort: %d, red: %d", color_sorting_enabled, redSide);
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
		// Arcade control scheme
		// Sets right motor voltage
		pros::delay(20); // Run for 20 ms then update
	}
}