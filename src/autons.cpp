#include "autons.hpp"
#include "definitions.hpp"
#include "pros/rtos.hpp"
#include <sys/_intsup.h>

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

void testAuton() {
    robot->turnToHeading(-90, 50000);
}


void goalSideRed() {
    color_sorting_enabled = false;
    redSide = true;
    autoSelected = true;
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, -32, 5000, {.forwards = false});
    robot->turnToPoint(3, -40, 1000, {.forwards = false, .maxSpeed = 80});
    robot->moveToPoint(7, -45, 5000, {.forwards = false, .maxSpeed = 72});
    pros::delay(700);  
    mogoState = true;
    mogo1.set_value(mogoState);
    mogo2.set_value(mogoState);
    intake.move(127);
    robot->turnToPoint(12, -33, 1000);
    robot->moveToPoint(12, -3, 5000, {}, false);
    while (robot->isInMotion()) {}
    robot->moveToPoint(12, -35, 2500, {.forwards = false});
    pros::delay(800);
    mogoState = false;
    mogo1.set_value(mogoState);
    mogo2.set_value(mogoState);
        // robot->turnToPoint(48, -38, 2000);
        // robot->moveToPoint(48, -38, 5000);
    // robot->turnToPoint(28, -32, 2500, {.forwards = false}, false);
    // intake.move(0);
    // robot->moveToPoint(28, -32, 5000, {.forwards = false}, false);
    // pros::delay(350);
    // mogoState = true;
    // mogo1.set_value(mogoState);
    // mogo2.set_value(mogoState);
    // intakeLift.set_value(true);
    // robot->moveToPose(56, -12, 90, 5000, {}, false);
    // intake.move(127);
    // intakeLift.set_value(false);
    // robot->moveToPoint(48, -12, 5000, {.forwards = false});

}
void goalSideBlue() {
    color_sorting_enabled = false;
    redSide = false;
    autoSelected = true;
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, -32, 5000, {.forwards = false});
    robot->turnToPoint(-3, -40, 1000, {.forwards = false, .maxSpeed = 80});
    robot->moveToPoint(-7, -45, 5000, {.forwards = false, .maxSpeed = 72});
    pros::delay(700);  
    mogoState = true;
    mogo1.set_value(mogoState);
    mogo2.set_value(mogoState);
    intake.move(127);
    robot->turnToPoint(-12, -33, 1000);
    robot->moveToPoint(-12, -32, 5000, {}, false);
    while (robot->isInMotion()) {}
    robot->moveToPoint(-12, -35, 2500, {.forwards = false});
    pros::delay(800);
    mogoState = false;
    mogo1.set_value(mogoState);
    mogo2.set_value(mogoState);
        // robot->turnToPoint(-48, -38, 2000);
        // robot->moveToPoint(-48, -38, 5000);
}
void ringSideRed(){
    
}
void ringSideBlue() {
    pros::lcd::print(4, "ring Side blue");
}
void skills(){
    pros::lcd::print(4, "skills");
}