#include "definitions.hpp"
#include "autons.hpp"
#include "pros/adi.hpp"


// Sensor and motor initialization
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Optical colorSensor(11);
pros::Motor intake(-19, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup ladyBrown({7, -8}, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::deg); 
pros::Rotation ladyBrownRotation(2);
pros::adi::DigitalOut mogo('A');
bool mogoState = LOW;

pros::adi::DigitalOut hang('E');
pros::adi::DigitalOut doinker('H');
bool hangState = LOW;

bool autoSelected = false;

bool doinkerState = LOW;

pros::adi::DigitalOut intakeLift('D');


pros::MotorGroup leftSide({1, -12, -13}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup rightSide({-4, 16, 9}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);

// Drivetrain setup
lemlib::Drivetrain drivetrain(&leftSide, &rightSide, 13.5, 3.25, 450, 8);

// Lateral PID controller configuration
lemlib::ControllerSettings lateral_controller(
    8, 
    0, 
    12, 
    0, 
    1, 
    100, 
    3, 
    500, 
    20);

// Angular PID controller configuration
lemlib::ControllerSettings angular_controller(
    2, 
    0, 
    5, 
    0, 
    1, 
    100, 
    3, 
    500, 
    0);

// Sensors
pros::Rotation horizontal_sensor(18);
pros::Rotation vertical_sensor(20);
pros::Imu imu(17);

// Tracking wheels setup
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_2, 0);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, -5);

// Odometer setup
lemlib::OdomSensors odom(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// Chassis setup
std::shared_ptr<lemlib::Chassis> robot(new lemlib::Chassis(drivetrain, lateral_controller, angular_controller, odom));

// Motion profile constraints
const double MAX_SPEED = 450 * (3.25 * M_PI) / 60;
RTMotionProfile::Constraints mp_constraints(
    MAX_SPEED,
    MAX_SPEED * 3, 
    0.1, 
    MAX_SPEED * 3, 
    MAX_SPEED * 100, 
    12
);

// Motion profile generator
std::shared_ptr<RTMotionProfile::ProfileGenerator> generator(new RTMotionProfile::ProfileGenerator(
    std::make_shared<RTMotionProfile::Constraints>(mp_constraints),
    0.1
));

// Ramsete controller setup
std::shared_ptr<RamseteController> ramsete(new RamseteController(
    robot, 
    generator,
    2.0,
    0.7
));

// Global variables initialization
bool stopIntakeControl = false;
bool redSide = false;
bool color_sorting_enabled = true;

// Autonomous selector map initialization

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

bool arm_in_load_pos = false;

std::map<int, std::pair<std::string, std::function<void()>>> autonSelectorMap = {
    {1, {"Goal Side Red", goalSideRed}},
    {2, {"Goal Side Blue", goalSideBlue}},
    {3, {"Ring Side Red", ringSideRed}},
    {4, {"Ring Side Blue", ringSideBlue}},
    {5, {"Skills", skills}},  

};

// Arm position constants
const double BASE_ARM_POS = 340;
const double LOAD_ARM_POS = 5.71;
const double SCORE_ARM_POS = 120;
 
int currentAutoSelection = 5;

