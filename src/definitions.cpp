#include "definitions.hpp"
#include "autons.hpp"

// Sensor and motor initialization
pros::Optical colorSensor(11);
pros::Motor intake(19, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup ladyBrown({5, -6}, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::deg); 
pros::Rotation ladyBrownRotation(2);
pros::adi::DigitalOut mogo1('A');
bool mogoState = LOW;
pros::adi::DigitalOut mogo2('B');

pros::adi::DigitalOut hang1('C');
pros::adi::DigitalOut hang2('D');
bool hangState = LOW;

pros::MotorGroup leftSide({1, 12, 13}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup rightSide({4, 7, 9}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);

// Drivetrain setup
lemlib::Drivetrain drivetrain(&leftSide, &rightSide, -11.5, 4.125, 200, 8);

// Lateral PID controller configuration
lemlib::ControllerSettings lateral_controller(20, 0, 3, 0, 1, 100, 3, 500, 20);

// Angular PID controller configuration
lemlib::ControllerSettings angular_controller(2, 0, 1, 3, 2, 100, 5, 500, 0);

// Sensors
pros::Rotation horizontal_sensor(4);
pros::Rotation vertical_sensor(4);
pros::Imu imu(10);

// Tracking wheels setup
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_2, -2.5);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, -5.5);

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
std::map<int, std::pair<std::string, std::function<void()>>> autonSelectorMap = {
    {1, {"Goal Side Red", goalSideRed}},
    {2, {"Goal Side Blue", goalSideBlue}},
    {3, {"Ring Side Red", ringSideRed}},
    {4, {"RIng Side Blue", ringSideBlue}},
    {5, {"Skills", skills}},  

};

bool arm_in_load_pos = false;

// Arm position constants
const double BASE_ARM_POS = 340;
const double LOAD_ARM_POS = 3.67;
const double SCORE_ARM_POS = 120;

int currentAutoSelection = -1;

