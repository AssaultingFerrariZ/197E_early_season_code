#include "autons.hpp"
#include "definitions.hpp"
#include "lemlib/asset.hpp"
#include "liblvgl/draw/lv_draw.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <sys/_intsup.h>

void wait(int ms) {
    pros::delay(ms);
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

void testAuton() {
    ladyBrown.set_voltage_limit_all(12000);
    arm_in_load_pos = true;
    robot->turnToHeading(-90, 50000);
    retractArm();
}


void goalSideRed() {
    color_sorting_enabled = false;
    redSide = true;
    autoSelected = true;
    robot->setPose(0, 0, 0);

    intake.move(0);
    robot->moveToPoint(0, -22, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 3});
    intake.move(0);
    robot->turnToPoint(4, -40, 1000, {.forwards = false});
    intake.move(0);
    robot->moveToPoint(7.5, -45, 5000, {.forwards = false, .maxSpeed = 72});
    intake.move(0);
    wait(800);
    mogo.set_value(true);
    pros::delay(300);
    robot->turnToPoint(12, -33, 5000, {.minSpeed = 20, .earlyExitRange = 3});
    intake.move(127);
    robot->moveToPoint(12, -35, 5000, {}, false);
    wait(1000);
    mogo.set_value(false);
    intake.move(0);
    robot->moveToPoint(12, -24, 5000);
    robot->turnToPoint(36, -36, 1000, {.forwards = false});
    robot->moveToPoint(28, -28, 5000, {.forwards = false, .maxSpeed = 90});
    robot->waitUntil(16);
    mogo.set_value(true);
    intake.move(-127);
    wait(350);
    intake.move(0);
    robot->turnToPoint(60, -10, 1000);
    intakeLift.set_value(true);
    intake.move(127);
    robot->moveToPoint(55, -10, 5000, {.minSpeed = 30, .earlyExitRange = 35});
    robot->moveToPoint(55, -10, 5000, {.maxSpeed = 20}, false);
    wait(250);
    intakeLift.set_value(false);
    robot->moveToPoint(40, -20, 5000, {.forwards = false, .maxSpeed = 50});
    robot->turnToPoint(44, -40, 1000);
    robot->moveToPoint(44, -40, 5000);

}
void goalSideBlue() {
    color_sorting_enabled = false;
    redSide = false;
    autoSelected = true;
    robot->setPose(0, 0, 0);

    intake.move(0);
    robot->moveToPoint(0, -22, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 3});
    intake.move(0);
    robot->turnToPoint(-3.5, -40, 1000, {.forwards = false});
    intake.move(0);
    robot->moveToPoint(-8, -47, 5000, {.forwards = false, .maxSpeed = 72});
    intake.move(0);
    wait(750);
    mogo.set_value(true);
    pros::delay(300);
    robot->turnToPoint(-12, -33, 5000, {.minSpeed = 20, .earlyExitRange = 3});
    intake.move(127);
    robot->moveToPoint(-12, -35, 5000, {}, false);
    wait(1000);
    mogo.set_value(false);
    intake.move(0);
    robot->moveToPoint(-12, -24, 5000);
    robot->turnToPoint(-36, -36, 1000, {.forwards = false});
    robot->moveToPoint(-28, -30, 5000, {.forwards = false, .maxSpeed = 90}, false);
    mogo.set_value(true);
    intake.move(-127);
    wait(350);
    intake.move(0);
    robot->turnToPoint(-60, -10, 1000);
    intakeLift.set_value(true);
    intake.move(127);
    robot->moveToPoint(-54, -12, 5000, {.minSpeed = 30, .earlyExitRange = 35});
    robot->moveToPoint(-54, -12, 5000, {.maxSpeed = 20}, false);
    wait(250);
    intakeLift.set_value(false);
    robot->moveToPoint(-40, -20, 5000, {.forwards = false});
    robot->turnToPoint(-44, -40, 1000);
    robot->moveToPoint(-44, -40, 5000);


}
void ringSideRed(){
    color_sorting_enabled = false;
    redSide = false;
    robot->setPose(0, 0, 45);
    pros::lcd::print(4, "ring side blue");
    robot->moveToPoint(-18, -18, 5000, {.forwards = false, .maxSpeed = 72}, false);
    mogo.set_value(true);
    robot->turnToPoint(3, -28, 1000);
    pros::delay(250);
    intake.move(127);
    robot->moveToPoint(0, -28, 5000);
    robot->turnToPoint(-7, -40, 1000);
    robot->moveToPoint(-5, -43, 5000);
    robot->moveToPoint(-5, -28, 5000, {.forwards = false});
    robot->moveToPose(2, -49, 180, 5000);
    robot->moveToPoint(2, -25, 5000, {.forwards = false}, false);
    intake.move(0);
    robot->turnToPoint(-36, -25, 1000, {.forwards = false});
    robot->moveToPoint(-43.65, -33.65, 5000, {.forwards = false});
}
void ringSideBlue() {
    color_sorting_enabled = false;
    redSide = false;
    robot->setPose(0, 0, -45);
    pros::lcd::print(4, "ring side blue");
    robot->moveToPoint(18, -18, 5000, {.forwards = false, .maxSpeed = 72}, false);
    mogo.set_value(true);
    robot->turnToPoint(-3, -28, 1000);
    pros::delay(250);
    intake.move(127);
    robot->moveToPoint(0, -28, 5000);
    robot->turnToPoint(7, -40, 1000);
    robot->moveToPoint(5, -43, 5000);
    robot->moveToPoint(5, -28, 5000, {.forwards = false});
    robot->moveToPose(-2, -49, 175, 5000);
    robot->moveToPoint(-2, -25, 5000, {.forwards = false}, false);
    intake.move(0);
    robot->turnToPoint(36, -25, 1000, {.forwards = false});
    robot->moveToPoint(40.25, -32.25, 5000, {.forwards = false});
}



void skills(){
    color_sorting_enabled = false;
    pros::lcd::print(4, "skills");
    robot->setPose(0, 0, 0);
    intake.move(127);
    wait(500);
    robot->moveToPoint(0, 10.974, 5000, {}, false);
    intake.move(0);
    robot->turnToPoint(-20.183, 10.974, 1000, {.forwards = false});
    robot->moveToPoint(-20.183, 10.974, 5000, {.forwards = false});
    robot->waitUntil(12.3);
    mogo.set_value(true);
    wait(500);
    robot->turnToPoint(-23.066, 37.722, 1000);
    intake.move(127);
    robot->moveToPoint(-23.066, 37.722, 5000);  
    wait(500);
    robot->turnToPoint(-46.852, 38.203, 1000);
    robot->moveToPoint(-46.852, 38.203, 5000, {}, false);
    wait(500);
    robot->turnToPoint(-52, -4.085, 1000);
    robot->moveToPoint(-52, 5, 5000, {.maxSpeed = 80}, false);
    wait(700);
    robot->turnToPoint(-60.749, 21.624, 1000);
    robot->moveToPoint(-60.749, 21.624, 5000);

    robot->turnToPoint(-67.275, -2.883, 1575, {.forwards = false, .maxSpeed = 80}, false);
    robot->moveToPoint(-67.275, -2.883, 1075, {.forwards = false, .maxSpeed = 80}, false);
    mogo.set_value(false);
    intake.move(0);

    robot->moveToPoint(1.526, 11.897, 5000, {.maxSpeed = 90});

    robot->turnToPoint(18.501, 11.897, 1500, {.forwards = false, .maxSpeed = 70});
    robot->moveToPoint(18.501, 11.897, 5000, {.forwards = false, .maxSpeed = 70});
    robot->waitUntil(35);
    mogo.set_value(true);

    robot->turnToPoint(23.787, 38.203, 1000);
    intake.move(127);
    robot->moveToPoint(23.787, 38.203, 1000);

    robot->turnToPoint(46.372, 38.443, 1000);
    robot->moveToPoint(46.372, 38.443, 5000);

    robot->turnToPoint(46.131, 2.883, 1000);
    robot->moveToPoint(46.131, 2.883, 5000, {.maxSpeed = 70}, false);
    wait(700);
    robot->turnToPoint(62.625, 16.897, 1000);
    robot->moveToPoint(62.625, 16.897, 5000);  
    robot->moveToPoint(70.554, -1.325, 5000, {.forwards = false, .maxSpeed = 90}, false);
    mogo.set_value(false);

    robot->moveToPoint(55.432, 7.993, 5000);
    robot->turnToPoint(40.529, 90.164, 1000, {.forwards = false});
    robot->moveToPoint(40.529, 90.164, 5000, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, .earlyExitRange = 3});

    robot->moveToPoint(18.671, 109.866, 5000, {.forwards = false});

    robot->turnToPoint(4.015, 109.866, 1000, {.forwards = false});
    robot->moveToPoint(4.015, 109.866, 5000, {.forwards = false});

    robot->waitUntil(12.5);
    mogo.set_value(true);

    robot->turnToPoint(23.957, 86.32, 1000);
    intake.move(127);
    robot->moveToPoint(23.957, 86.32, 1000);

    robot->turnToPoint(45.822, 85.839, 1000);
    robot->moveToPoint(45.822, 85.839, 5000);

    robot->turnToPoint(45.341, 110.347, 1000);
    robot->moveToPoint(45.341, 110.347, 5000);

    robot->turnToPoint(59.037, 110.347, 1000);
    robot->moveToPoint(59.037, 110.347, 5000);

    robot->turnToPoint(46.302, 120.919, 1000);
    robot->moveToPoint(46.302, 120.919, 5000);

    robot->moveToPoint(64.322, 125.964, 5000, {.forwards = false, .maxSpeed = 60}, false);
    mogo.set_value(false);

    robot->moveToPoint(4.015, 109.866, 5000);

    robot->turnToPoint(24.015, 120.919, 1000);
    robot->moveToPoint(24.015, 120.919, 5000);

    robot->turnToHeading(-90, 1000);
    robot->moveToPoint(70.105, 120.919, 3000);
    robot->get_drivetrain().leftMotors->move(-60);
    robot->get_drivetrain().rightMotors->move(-60);
}