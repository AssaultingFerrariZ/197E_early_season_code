#include "definitions.hpp"
#include "autons.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"

// Sensor and motor initialization
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Optical colorSensor(15);
pros::Motor intake(-8, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup ladyBrown({-1, 3}, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::deg); 
pros::Rotation ladyBrownRotation(2);
pros::adi::DigitalOut mogo('C');
bool mogoState = LOW;

pros::adi::DigitalOut hang('E');
pros::adi::DigitalOut doinker('B');
bool hangState = LOW;

bool autoSelected = false;
bool autoActive = false;

bool doinkerState = LOW;

pros::adi::DigitalOut intakeLift('A');

pros::Distance intakeDistance(16);

pros::MotorGroup leftSide({20, -11, -13}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
pros::MotorGroup rightSide({-18, 6, 17}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);

// Drivetrain setup
lemlib::Drivetrain drivetrain(&leftSide, &rightSide, 13.5, 3.25, 450, 8);

// Lateral PID controller configuration
lemlib::ControllerSettings lateral_controller(
    8, 
    0, 
    12.5, 
    0, 
    1, 
    300, 
    3, 
    750, 
    20);

// Angular PID controller configuration
lemlib::ControllerSettings angular_controller(
    2.25, 
    0.1, 
    19.225, 
    7, 
    0.5, 
    300, 
    1.5, 
    500, 
    0);

// Sensors
pros::Rotation horizontal_sensor(4);
pros::Rotation vertical_sensor(5);
pros::Imu imu(19);

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
RingColor redSide = UNKNOWN;
bool color_sorting_enabled = true;

// Autonomous selector map initialization}

std::map<int, std::pair<std::string, std::function<void()>>> autonSelectorMap = {
    {1, {"Goal Side Red", goalSideRed}},
    {2, {"Goal Side Blue", goalSideBlue}},
    {3, {"Ring Side Red", ringSideRed}},
    {4, {"Ring Side Blue", ringSideBlue}},
    {5, {"Safe Left Red", safeAutoLeftRed}},
    {6, {"Safe Left Blue", safeAutoLeftBlue}},
    {7, {"Safe Right Red", safeAutoRightRed}},
    {8, {"Safe Right Blue", safeAutoRightBlue}},
    {9, {"Solo AWP Red", soloAWPRed}},
    {10, {"Solo AWP Blue", soloAWPBlue}},
    {11, {"Skills", skills}}, 

};

// Arm position constants
const double BASE_ARM_POS = 5.00;
const double LOAD_ARM_POS = 33.09;

bool autoHasBeenActive = false;

double arm_target = BASE_ARM_POS;
bool arm_pid_enabled = true;

int currentAutoSelection = 11;

pros::adi::DigitalOut odomRetract('D');

void decelerate(float x, float y, float decelerateSpeed = 20, float earlyExitRange = 0.5, bool async = false) {
    robot->moveToPoint(x, y, 5000, {.forwards = false, .minSpeed = decelerateSpeed, .earlyExitRange = earlyExitRange});
    robot->moveToPoint(x, y, 5000, {.forwards = false, .maxSpeed = decelerateSpeed}, async);
}