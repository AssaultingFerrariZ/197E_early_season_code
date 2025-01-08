#include "definitions.hpp"
#include "autons.hpp"
#include "pros/adi.hpp"


// Sensor and motor initialization
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Optical colorSensor(11);
pros::Motor intake(-19, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup ladyBrown({18, -1}, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::deg); 
pros::Rotation ladyBrownRotation(14);
pros::adi::DigitalOut mogo('A');
bool mogoState = LOW;

pros::adi::DigitalOut hang('E');
pros::adi::DigitalOut doinker('H');
bool hangState = LOW;

bool autoSelected = false;

bool doinkerState = LOW;

pros::adi::DigitalOut intakeLift('D');


pros::MotorGroup leftSide({15, -12, -10}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
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
    3, 
    100, 
    5, 
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

void moveArm(double angle) {
	//define PID constants
	static lemlib::PID armPID(4, 0.5, 0, 2, false);
	//reset integral upon execution
	armPID.reset();
	double time = 0;
	//killswitch to block arm from getting stuck
	while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		//target varies based on the arm's current position
		double error = (angle - ladyBrownRotation.get_angle() / 100.0); 
			//centidegrees to degrees
		double output = clamp(error, -127, 127);
		ladyBrown.move(output);
		
		//exit the PID once within angle tolerance or timeout is reached
		if (fabs(error) <= 1.4 || time > 1000) { 
			break; 
		}

		time += 10; 
		pros::delay(10);

	}
	
}

std::map<int, std::pair<std::string, std::function<void()>>> autonSelectorMap = {
    {1, {"Goal Side Red", goalSideRed}},
    {2, {"Goal Side Blue", goalSideBlue}},
    {3, {"Ring Side Red", ringSideRed}},
    {4, {"Ring Side Blue", ringSideBlue}},
    {5, {"Skills", skills}},  

};

// Arm position constants
const double BASE_ARM_POS = 25.22;
const double LOAD_ARM_POS = 51.15;
 
int currentAutoSelection = 5;

