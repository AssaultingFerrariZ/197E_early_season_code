#include "autons.hpp"
#include "definitions.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/draw/lv_draw.h"
#include "pros/llemu.hpp"
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



void goalSideRed() {
    // color_sorting_enabled = true;
    // redSide = true;
    // autoSelected = true;
    // robot->setPose(0, 0, 0);

    // //Initial goal grab
    // robot->moveToPoint(0, -22, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 3});
    // robot->turnToPoint(4, -40, 1000, {.forwards = false});
    // robot->moveToPoint(7, -45, 5000, {.forwards = false, .maxSpeed = 62});
    // robot->waitUntil(21);
    // mogo.set_value(true);
    // wait(500);
    // // //Grab ring at bottom of stack + score preload
    // robot->turnToPoint(10, -33, 5000, {.minSpeed = 20, .earlyExitRange = 3});
    // robot->moveToPoint(10, -35, 5000, {});
    // intake.move(127);
    // robot->waitUntilDone();
    // wait(1000);
    // mogo.set_value(false);
    // intake.move(0);

    // // //go for second goal
    // robot->moveToPoint(10, -30, 5000);
    // robot->turnToPoint(27, -30, 1000, {.forwards = false});
    // robot->moveToPoint(27, -30, 5000, {.forwards = false, .maxSpeed = 72});
    // robot->waitUntil(16.5);
    // mogo.set_value(true);

    // // //outtake any potential red rings
    // intake.move(-127);
    // wait(250);

    // // //go for ring at top of stack
    // robot->turnToPoint(60, -10, 1000);
    // intakeLift.set_value(true);
    // intake.move(127);
    
    // robot->moveToPoint(54, -10, 5000, {.maxSpeed = 60}, false);
    // wait(250);
    
    // // //drop the piston to be able to intake the bottom ring
    // intakeLift.set_value(false);
    // wait(500);
    // robot->moveToPoint(40, -20, 5000, {.forwards = false});
    // robot->turnToPoint(40, -45, 1000);

    // // //touch the bar for win point
    // robot->moveToPoint(40, -42, 5000);

    color_sorting_enabled = false;
    redSide = true;
    autoSelected = true;
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, 20.319, 2000, {.minSpeed = 90, .earlyExitRange = 2});
    doinker.set_value(true);
    robot->moveToPoint(-1.693, 36.223, 3000, {.maxSpeed = 90}, false);
    doinker.set_value(false);
    robot->moveToPoint(0, 16.319, 2500, {.forwards = false}, false);
    doinker.set_value(true);
    wait(750);
    robot->moveToPoint(0, 5.319, 1000, {.forwards = false}, false);
    doinker.set_value(false);
    robot->turnToHeading(160, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    robot->moveToPoint(1.441, 32.294, 5000, {.forwards = false, .maxSpeed = 60}, false);
    mogo.set_value(true);
    robot->moveToPoint(6.046, -0.961, 5000);
    wait(100);
    intake.move(127);
    robot->waitUntilDone();
    intake.move(0);
    doinker.set_value(true);
    wait(500);
    robot->turnToHeading(0, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    robot->waitUntil(45);
    doinker.set_value(false);
    robot->waitUntilDone();
    mogo.set_value(false);
    robot->moveToPose(-20.917, 38.254, -100, 5000, {.lead = 0.3}, false);
    robot->moveToPoint(-23.917, 38.254, 5000, {.maxSpeed = 30});
    intake.move(100);
    wait(500);
    intake.move(0);
    robot->moveToPoint(-20.917, 38.254, 1000, {.forwards = false});
    robot->turnToPoint(-42.118, 27.319, 1500, {.forwards = false});
    robot->moveToPoint(-42.118, 27.319,  5000, {.forwards = false, .maxSpeed = 80}, false);
    mogo.set_value(true);
    intake.move(127);
    robot->turnToHeading(45, 1000);
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, 10, 1000);


}

/* //go for rings in middle
    robot->turnToPoint(7.403, -40.859, 1000);
    wait(250);
    intake.move(127);
    robot->moveToPose(7.403, -39.859, 160, 5000); //first ring
    robot->turnToPoint(16.593, -43.582, 1000);
    robot->moveToPoint(16.593, -43.582, 5000, {}); //second ring

    //grab 4th ring
    robot->moveToPoint(-2.162, -39.884, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 3});
    robot->turnToPoint(11.208, -27.56, 1000, {});
    robot->moveToPoint(11.208, -27.56, 5000, {}, false);
    pros::delay(1000);
*/
void goalSideBlue() {
   color_sorting_enabled = true;
    redSide = true;
    autoSelected = true;
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, 20.319, 2000, {.minSpeed = 90, .earlyExitRange = 2});
    doinker.set_value(true);
    robot->moveToPoint(-2.193, 36.223, 3000, {.maxSpeed = 90}, false);
    doinker.set_value(false);
    robot->moveToPoint(-3.5, 15, 2500, {.forwards = false}, false);
    doinker.set_value(true);
    wait(750);
    robot->moveToPoint(-3.5, 5.319, 1000, {.forwards = false}, false);
    doinker.set_value(false);
    robot->turnToHeading(160, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    robot->moveToPoint(-0.441, 38.694, 5000, {.forwards = false, .maxSpeed = 70}, false);
    wait(100);
    mogo.set_value(true);
    wait(100);
    intake.move(127);
    robot->moveToPoint(0, 5, 5000);
    robot->turnToHeading(90, 1000, {}, false);
    intake.move(0);
    mogo.set_value(false);
    robot->turnToPoint(14, 30, 1000, {.forwards = false});
    robot->moveToPoint(14, 30, 1000, {.forwards = false, .maxSpeed = 50}, false);
    mogo.set_value(true);
    wait(100);
    robot->turnToPoint(-6, 30, 1000);
    robot->moveToPoint(-6, 30, 5000);
    intake.move(127);


}
void ringSideRed() {
    color_sorting_enabled = true;
    redSide = true;

    //ensure the robot is driving backwards to the goal
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, -23, 5000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 12});
    robot->moveToPoint(0, -23, 5000, {.forwards = false, .maxSpeed = 20}, false);  //decelarate
    robot->setPose(-7.695, -19.737, 28);
    mogo.set_value(true);
    wait(250);

    //go for rings in middle
    robot->turnToPoint(7.403, -40.859, 1000);
    wait(250);
    intake.move(127);
    robot->moveToPose(9.403, -42.859, 160, 5000, {}, false); //first ring

    //grab 4th ring
    robot->moveToPoint(-4.162, -36.884, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 3});
    robot->turnToPoint(13.208, -26.06, 1000, {});
    robot->moveToPoint(13.208, -26.06, 5000, {}, false);
    robot->turnToPoint(18.593, -43.582, 1000);
    robot->moveToPoint(18.593, -43.582, 5000, {}, false); //second ring
    intake.move(0);
    robot->moveToPoint(11.203, -27.56, 5000, {.forwards = false}, false);
    intake.move(127);

    //grab 5th ring
    robot->turnToPoint(-32.404, -6.773, 1000, {.maxSpeed = 120}, false);
    intakeLift.set_value(true);
    robot->moveToPoint(-32.18, -7.01, 5000, {.minSpeed = 75, .earlyExitRange = 32});
    robot->moveToPoint(-32.18, -7.01, 5000, {.maxSpeed = 45}, false);
    intakeLift.set_value(false);

    //touch bar
    robot->moveToPoint(-20, -6, 5000, {.forwards = false}, false);
    wait(250);
    robot->turnToPoint(-20, -3, 1000);
    robot->moveToPoint(-20, -32, 5000, {});



}
void ringSideBlue() {
    color_sorting_enabled = true;
    redSide = false;

    //ensure the robot is driving backwards to the goal
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, -23, 5000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 12});
    robot->moveToPoint(0, -23, 5000, {.forwards = false, .maxSpeed = 20}, false);  //decelarate
    robot->setPose(7.695, -19.737, -28);
    mogo.set_value(true);
    wait(250);

    //go for rings in middle
    robot->turnToPoint(-7.403, -40.859, 1000);
    wait(250);
    intake.move(127);
    robot->moveToPose(-9.403, -42.859, -160, 5000, {}, false); //first ring

    //grab 4th ring
    robot->moveToPoint(4.162, -36.884, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 3});
    robot->turnToPoint(-13.208, -26.06, 1000, {});
    robot->moveToPoint(-13.208, -26.06, 5000, {}, false);
    robot->turnToPoint(-18.593, -40.582, 1000);
    robot->moveToPoint(-18.593, -40.582, 5000, {}, false); //second ring
    intake.move(0);
    robot->moveToPoint(-11.203, -27.56, 5000, {.forwards = false}, false);
    intake.move(127);

    //grab 5th ring
    // robot->turnToPoint(32.404, -6.773, 1000, {.maxSpeed = 120}, false);
    // intakeLift.set_value(true);
    // robot->moveToPoint(35.18, -7.01, 5000, {.minSpeed = 75, .earlyExitRange = 32});
    // robot->moveToPoint(35.18, -7.01, 5000, {.maxSpeed = 45}, false);
    // intakeLift.set_value(false);

    // //touch bar
    // robot->moveToPoint(20, -6, 5000, {.forwards = false}, false);
    // wait(250);
    // robot->turnToPoint(20, -32, 1000);
    // robot->moveToPoint(20, -36, 5000, {});
}

void safeAutoLeft() {
    color_sorting_enabled = false;
    autoSelected = true;
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, 5, 5000);
    moveArm(200);
    robot->moveToPoint(-5, -16, 5000, {.forwards = false});
    robot->turnToPoint(-21, -27, 1000, {.forwards = false});
    robot->moveToPoint(-21, -27, 1000, {.forwards = false, .maxSpeed = 60}, false);
    mogo.set_value(true);
    robot->turnToPoint(-12, -47, 1000);
    robot->moveToPoint(-12, -47, 5000);
    intake.move(127);

}

void safeAutoRight() {
    color_sorting_enabled = false;
    autoSelected = true;
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, 5, 5000);
}

void skills(){
    color_sorting_enabled = false;
    pros::lcd::print(4, "skills");
    robot->setPose(0, 0, 0);
    intake.move(127);
    wait(500);
    robot->moveToPoint(0, 12.274, 5000, {}, false);
    intake.move(0);
    robot->turnToPoint(-18.183, 12.274, 1000, {.forwards = false});
    robot->moveToPoint(-18.183, 12.274, 5000, {.forwards = false, .maxSpeed = 70});
    robot->waitUntil(17.3);
    mogo.set_value(true);
    wait(500);
    robot->turnToPoint(-23.066, 37.722, 1000);
    intake.move(127);
    robot->moveToPoint(-23.066, 37.722, 5000);  
    wait(500);
    robot->turnToPoint(-46.852, 38.203, 1000);
    robot->moveToPoint(-46.852, 38.203, 5000, {}, false);
    wait(500);
    robot->turnToPoint(-46, -4.085, 1000);
    robot->moveToPoint(-46, -4.085, 5000, {.maxSpeed = 50}, false);
    wait(700);
    robot->turnToPoint(-60.749, 21.624, 1000);
    robot->moveToPoint(-60.749, 21.624, 5000);

    robot->turnToPoint(-65.275, 0.883, 1575, {.forwards = false, .maxSpeed = 80}, false);
    robot->moveToPoint(-65.275, 0.883, 1075, {.forwards = false, .maxSpeed = 80}, false);
    mogo.set_value(false);
    intake.move(0);

    robot->moveToPoint(1.526, 11.897, 5000, {.maxSpeed = 90});

    robot->turnToPoint(18.501, 11.897, 1500, {.forwards = false, .maxSpeed = 70});
    robot->moveToPoint(18.501, 11.897, 5000, {.forwards = false, .maxSpeed = 70});
    robot->waitUntil(17);
    mogo.set_value(true);
    wait(500);
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
    //DILAN WAS HERE
    intake.move(-127);
    wait(500);
    intake.move(0);
    robot->moveToPoint(55.432, 7.993, 5000); 
}