#include "autons.hpp"
#include "definitions.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/draw/lv_draw.h"
#include "liblvgl/misc/lv_area.h"
#include "pros/device.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <sys/_intsup.h>
#include <thread>

//while the arm is less than deg degrees away from its target, it will wait
//e.x. if deg = 50, then it will wait until the arm is 50 degrees 
void armWaitUntil(int deg) {
    while (fabs(arm_target - ladyBrownRotation.get_angle() / 100.0) < deg) {
        pros::delay(10);
    }
}

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
    color_sorting_enabled = true;
    redSide = RED;
    autoSelected = true;
    robot->setPose(0, 0, 0);

    //begin the forward movement to the goal at max speed
    robot->moveToPoint(0, 20.319, 2000, {.minSpeed = 100, .earlyExitRange = 2});
    doinker.set_value(true);

    //turn towards the goal in one movement
    robot->moveToPoint(-4, 39.923, 2500, {.maxSpeed = 70, .minSpeed = 30, .earlyExitRange = 0.5}, false);
    doinker.set_value(false); //pull back rush mech to pick up goal

    //pull back the goal without stopping (due to the motion chaining in the previous motion)
    robot->moveToPoint(-1.25, 15.319, 2500, {.forwards = false, .maxSpeed = 100}, false);

    //release the goal
    doinker.set_value(true);
    wait(500); //allow goal to stabilize so the position does not vary

    //back away from goal and retract doinker
    robot->moveToPoint(-3, 8.319, 1000, {.forwards = false}, false);
    doinker.set_value(false);

    //turn to the goal counterclockwise so it does not hit the wall
    robot->turnToPoint(-1.5, 31.294, 1000, { .forwards = false, 
        .direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});

    //move towards the goal and clamp
    robot->moveToPoint(-1.5, 34.294, 2500, {.forwards = false, .maxSpeed = 70});
    wait(1000);
    mogo.set_value(true);

    //drive forward while scoring ring
    robot->moveToPoint(-10.046, 3.361, 1000, {});
    wait(150);
    intake.move(127);
    robot->waitUntilDone();
    mogo.set_value(false); //drops the goal in a safe place
    wait(250);
    //drive into corner while evading opposite color ring near the stack
    robot->turnToPoint(6.046, -16.361, 1000);
    robot->moveToPoint(6.046, -16.361, 1000);
       //stop ring in the intake with distance sensor
    pros::Task intStop([&] {
        int time = 0; //timeout to make sure intake does not wait indefinitely
        while (intakeDistance.get_distance() > 90 && time < 1950) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });
    robot->waitUntilDone();
    //back out of corner
    robot->moveToPoint(-6.046, -0.961, 1000, {.forwards = false});

    //go to mobile goal
    robot->moveToPoint(-35.518, 26.729, 5000, {.forwards = false, .maxSpeed = 75});
    robot->waitUntil(36);
    mogo.set_value(true);

    //wait a little bit before intaking so the hooks don't get stuck
    wait(250);
    intake.move(127);

    //grab final ring
    robot->turnToPoint(-18, 32, 1000);
    robot->moveToPoint(-18, 32, 2500);
    // arm_target = LOAD_ARM_POS;
    // robot->turnToPoint(9, 42, 1000, {.minSpeed = 40, .earlyExitRange = 1});
    // robot->moveToPoint(9, 42, 1000, {.maxSpeed = 90});
    // intake.move(0);
    // wait(100);
    // intake.move(127);
    // wait(100);
    // intake.move(0);
    // wait(100);
    // intake.move(127);
    // wait(100);
    // intake.move(0);
    // robot->waitUntilDone();
    // arm_target = 176.55;

}

void goalSideBlue() {
    //initialize auton, color sort, and position
    color_sorting_enabled = true;
    redSide = BLUE;
    autoSelected = true;
    robot->setPose(0, 0, 0);

    //same rush as the red side, but on the opposite side of the ring this time due to being on the left
    robot->moveToPoint(0, 20.319, 2000, {.minSpeed = 127, .earlyExitRange = 2});
    doinker.set_value(true);
    robot->moveToPoint(-2.193, 36.223, 3000, {.minSpeed = 30, .earlyExitRange = 0.5}, false);
    doinker.set_value(false);
    robot->moveToPoint(-3.5, 15, 2500, {.forwards = false}, false);
    doinker.set_value(true);
    robot->moveToPoint(-3.5, 5.319, 1000, {.forwards = false}, false);
    doinker.set_value(false);
    robot->turnToPoint(2.5, 35.694, 1000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE});
    robot->moveToPoint(2.5, 35.694, 5000, {.forwards = false, .maxSpeed = 80});
    robot->waitUntil(34);
    mogo.set_value(true);
    intake.move(127);
    robot->moveToPoint(0, 5, 5000, {.minSpeed = 15, .earlyExitRange = 6});
    robot->turnToHeading(0, 500, {}, false);
    intake.move(0);
    mogo.set_value(false);
    robot->turnToPoint(16, 29, 500, {.forwards = false});
    robot->moveToPoint(16, 29, 1000, {.forwards = false, .maxSpeed = 80});
    robot->waitUntil(24);
    mogo.set_value(true);
    wait(100);
    robot->turnToPoint(-6, 36, 1000);
    robot->moveToPoint(-6, 36, 5000, {});
    intake.move(127);
    robot->moveToPoint(-12, 34, 1000, {.minSpeed = 100, .earlyExitRange = 3});
    robot->moveToPoint(0, 0,  850, {.minSpeed = 85, .earlyExitRange = 3});
    robot->moveToPoint(-25, -14, 2000, {.maxSpeed = 85});
    robot->moveToPoint(10, 30, 2000, {.forwards = false});    
}

void ringSideRed() {
    color_sorting_enabled = true;
    redSide = RED;

    //ensure the robot is driving backwards to the goal
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, -23, 5000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 12});
    robot->moveToPoint(0, -23, 5000, {.forwards = false, .maxSpeed = 20}, false);  //decelarate
    robot->setPose(-7.695, -19.737, 28);
    mogo.set_value(true);

    //go for rings in middle
    robot->turnToPoint(7.403, -40.859, 1000);
    intake.move(127);
    robot->moveToPose(9.403, -42.859, 160, 5000, {}, false); //first ring

    //grab 4th ring
    robot->moveToPoint(-4.162, -36.884, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 3});
    robot->turnToPoint(13.208, -26.06, 1000, {.minSpeed = 30, .earlyExitRange = 1});
    robot->moveToPoint(13.208, -26.06, 5000, {}, false);
    robot->turnToPoint(16.593, -43.582, 1000);
    robot->moveToPoint(16.593, -43.582, 5000, {}, false); //second ring
    robot->moveToPoint(11.203, -27.56, 5000, {.forwards = false}, false);

    //grab 5th ring
    robot->turnToPoint(-32.404, -7.773, 1000, {}, false);
    intakeLift.set_value(true);
    robot->moveToPoint(-32.18, -7.01, 5000, {.minSpeed = 75, .earlyExitRange = 12});
    robot->moveToPoint(-32.18, -7.01, 5000, {.maxSpeed = 50}, false);
    intakeLift.set_value(false);

    //touch bar
    robot->turnToPoint(-22, -24, 1000, {.minSpeed = 20, .earlyExitRange = 0.5});
    robot->moveToPoint(-22, -35, 5000, {});



}
void ringSideBlue() {
    color_sorting_enabled = true;
    redSide = BLUE;

    //ensure the robot is driving backwards to the goal
    robot->setPose(0, 0, 0);
    robot->moveToPoint(0, -23, 5000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 12});
    robot->moveToPoint(0, -23, 5000, {.forwards = false, .maxSpeed = 20}, false);  //decelarate
    robot->setPose(7.695, -19.737, -28);
    mogo.set_value(true);

    //go for rings in middle
    robot->turnToPoint(-7.403, -40.859, 1000);
    intake.move(127);
    robot->moveToPose(-9.403, -42.859, -160, 5000, {}, false); //first ring

    //grab 4th ring
    robot->moveToPoint(4.162, -36.884, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 3});
    robot->turnToPoint(-13.208, -26.06, 1000, {.minSpeed = 30, .earlyExitRange = 1});
    robot->moveToPoint(-13.208, -26.06, 5000, {}, false);
    robot->turnToPoint(-16.593, -43.582, 1000);
    robot->moveToPoint(-16.593, -43.582, 5000, {}, false); //second ring
    robot->moveToPoint(-11.203, -27.56, 5000, {.forwards = false}, false);

    //grab 5th ring
    robot->turnToPoint(32.404, -7.773, 1000, {}, false);
    intakeLift.set_value(true);
    robot->moveToPoint(32.18, -7.01, 5000, {.minSpeed = 75, .earlyExitRange = 12});
    robot->moveToPoint(32.18, -7.01, 5000, {.maxSpeed = 50}, false);
    intakeLift.set_value(false);

    //touch bar
    robot->turnToPoint(22, -24, 1000, {.minSpeed = 20, .earlyExitRange = 0.5});
    robot->moveToPoint(22, -35, 5000, {});

}

void safeAutoLeftRed() {
    color_sorting_enabled = true;
    autoSelected = true;
    redSide = RED;
    robot->setPose(0, 0, 122.33);
    //puts it on the stake
    arm_target = 235;
    wait(1000);
    robot->moveToPoint(-12, 12, 5000, {.forwards = false});
    wait(500);
    arm_target = 0;
    robot->turnToPoint(10, 13, 5000);
    intakeLift.set_value(true);
    pros::Task intStop([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 4000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });
    robot->moveToPoint(5.5, 13, 5000, {.maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 12});
    robot->moveToPoint(5.5, 13, 5000, {.maxSpeed = 50}, false);
    intakeLift.set_value(false);
    robot->moveToPoint(-6, 13, 5000, {.forwards = false});
    robot->turnToPoint(-14.5, 34.88, 5000, {.forwards = false});
    robot->moveToPoint(-14.5, 34.88, 5000, {.forwards = false, .maxSpeed = 70});
    robot->waitUntil(24);
    mogo.set_value(true);
    wait(250);
    intake.move(127);
    wait(250);
    robot->moveToPoint(-32, 33.88, 5000);  
    pros::Task intStop2([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 1000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });
    // robot->moveToPoint(-18, 8, 5000, {.minSpeed = 60, .earlyExitRange = 12});
    robot->moveToPoint(-65, -18, 2500, {.minSpeed = 120});
    intake.move(127);
    // arm_target = 70;
    // robot->turnToPoint(-2, 38, 5000);
    //     pros::Task intStop3([&] {
    //     int time = 0;
    //     while (intakeDistance.get_distance() > 90 && time < 750) {
    //         intake.move(127);
    //         pros::delay(10);
    //         time += 10;
    //     }
    //     intake.move(0);
    // });
    robot->moveToPoint(-8, 45, 5000);
    intake.move(127);
    robot->waitUntil(15);
    arm_target += 70;
    
}

void safeAutoLeftBlue() {
    color_sorting_enabled = true;
    autoSelected = true;
    redSide = BLUE;
    robot->setPose(0, 0, 122.33);
    //puts it on the stake
    arm_target = 235;
    wait(1000);
    robot->moveToPoint(-12, 12, 5000, {.forwards = false});
    wait(500);
    arm_target = 0;
    robot->turnToPoint(10, 13, 5000);
    intakeLift.set_value(true);
    pros::Task intStop([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 4000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });
    robot->moveToPoint(5.5, 13, 5000, {.maxSpeed = 50}, false);
    intakeLift.set_value(false);
    robot->moveToPoint(-6, 13, 5000, {.forwards = false});
    robot->turnToPoint(-14.5, 34.88, 5000, {.forwards = false});
    robot->moveToPoint(-14.5, 34.88, 5000, {.forwards = false, .maxSpeed = 70});
    robot->waitUntil(24);
    mogo.set_value(true);
    wait(250);
    intake.move(127);
    wait(250);
    robot->moveToPoint(-32, 33.88, 5000);  
    pros::Task intStop2([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 1000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });
    // robot->moveToPoint(-18, 8, 5000, {.minSpeed = 60, .earlyExitRange = 12});
    robot->moveToPoint(-65, -18, 2500, {.minSpeed = 120});
    intake.move(127);
    // arm_target = 70;
    // robot->turnToPoint(-2, 38, 5000);
    //     pros::Task intStop3([&] {
    //     int time = 0;
    //     while (intakeDistance.get_distance() > 90 && time < 750) {
    //         intake.move(127);
    //         pros::delay(10);
    //         time += 10;
    //     }
    //     intake.move(0);
    // });
    robot->moveToPoint(-8, 45, 5000);
    intake.move(127);
    robot->waitUntil(15);
    arm_target += 70;
    

}

void safeAutoRightRed() {
    color_sorting_enabled = true;
    autoSelected = true;
    redSide = RED;
    robot->setPose(0, 0, -121.51);
    //puts it on the stake
    arm_target = 235;
    wait(1000);
    robot->moveToPoint(12, 12, 5000, {.forwards = false});
    wait(500);
    arm_target = 0;
    robot->turnToPoint(-10, 13, 5000);
    intakeLift.set_value(true);
    pros::Task intStop([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 4000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });
    robot->moveToPoint(-4.5, 13, 5000, {.maxSpeed = 50}, false);
    intakeLift.set_value(false);
    robot->moveToPoint(6, 13, 5000, {.forwards = false});
    robot->turnToPoint(14.5, 34.88, 5000, {.forwards = false});
    robot->moveToPoint(14.5, 34.88, 5000, {.forwards = false, .maxSpeed = 70});
    robot->waitUntil(24);
    mogo.set_value(true);
    wait(250);
    intake.move(127);
    wait(250);
    robot->moveToPoint(32, 33.88, 5000);  
    pros::Task intStop2([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 1000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });

    // robot->moveToPoint(18, 8, 5000, {.minSpeed = 60, .earlyExitRange = 12});
    robot->moveToPoint(65, -15, 2500, {.minSpeed = 120});
    intake.move(127);
    // wait(1250);
    // robot->cancelMotion();
    // wait(750);
    // arm_target = 70;
    // robot->turnToPoint(2, 38, 5000);
    robot->moveToPoint(8, 45, 5000);
    intake.move(-127);
    intake.move(127);
    robot->waitUntil(15);
    arm_target += 70;
    



    // arm_target = 0;

}

void safeAutoRightBlue() {
    color_sorting_enabled = true;
    autoSelected = true;
    redSide = BLUE;
    robot->setPose(0, 0, -121.51);
    //puts it on the stake
    arm_target = 235;
    wait(1000);
    robot->moveToPoint(12, 12, 5000, {.forwards = false});
    wait(500);
    arm_target = 0;
    robot->turnToPoint(-10, 13, 5000);
    intakeLift.set_value(true);
    pros::Task intStop([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 4000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });
    robot->moveToPoint(-4.5, 13, 5000, {.maxSpeed = 50}, false);
    intakeLift.set_value(false);
    robot->moveToPoint(6, 13, 5000, {.forwards = false});
    robot->turnToPoint(14.5, 34.88, 5000, {.forwards = false});
    robot->moveToPoint(14.5, 34.88, 5000, {.forwards = false, .maxSpeed = 70});
    robot->waitUntil(24);
    mogo.set_value(true);
    wait(250);
    intake.move(127);
    wait(250);
    robot->moveToPoint(32, 33.88, 5000);  
    pros::Task intStop2([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 1000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });

    // robot->moveToPoint(18, 8, 5000, {.minSpeed = 60, .earlyExitRange = 12});
    robot->moveToPoint(65, -15, 2500, {.minSpeed = 120});
    intake.move(127);
    // wait(1250);
    // robot->cancelMotion();
    // wait(750);
    // arm_target = 70;
    // robot->turnToPoint(2, 38, 5000);
    robot->moveToPoint(8, 45, 5000);
    intake.move(-127);
    intake.move(127);
    robot->waitUntil(15);
    arm_target += 70;
    



    // arm_target = 0;

}

void soloAWPRed() {
    //initialize color sort and position (initial start angle must be accounted for)
    color_sorting_enabled = true;
    autoSelected = true;
    redSide = RED;
    robot->setPose(0, 0, 122.33);

    //puts preload on the stake
    arm_target = 235;
    wait(1000);

    //back out, chaining the movement into the turn
    robot->moveToPoint(-12, 12, 5000, {.forwards = false, .minSpeed = 70, .earlyExitRange = 3});
    
    //reset arm position
    wait(250);
    arm_target = 0;

    //turn to the goal, drive forward, and clamp it
    robot->turnToPoint(-14.5, 37.88, 5000, {.forwards = false});
    robot->moveToPoint(-14.5, 37.88, 5000, {.forwards = false, .maxSpeed = 70});
    robot->waitUntil(26);
    mogo.set_value(true);

    //wait before moving the intake so hooks don't catch
    wait(250);
    intake.move(127);

    //intake close ring onto mobile goal
    robot->moveToPoint(-34, 33.88, 1000);

    //chain movements to get to the ring stack 
    robot->moveToPoint(-15, 13, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    robot->turnToPoint(20, 13, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    robot->moveToPoint(7, 13, 1000, {.minSpeed = 40, .earlyExitRange = 2});
    robot->waitUntil(4);

    //deposits the mobile goal, and travels to ring
    mogo.set_value(false);
    robot->moveToPoint(9, 13, 1000, {}, false);
    wait(500);
    robot->moveToPoint(32, 13, 1000, {});
    
    //intakes ring, but keeps it in the intake
    pros::Task intStop([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 5000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });

    //clamps the mobile goal
    robot->turnToPoint(34.5, 42, 5000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 1});
    robot->moveToPoint(34.5, 42, 5000, {.forwards = false, .maxSpeed = 80});
    robot->waitUntil(29);
    mogo.set_value(true);
    wait(250);

    //finishes intaking the ring, grabs another
    intake.move(127);
    wait(250);
    robot->turnToPoint(58.5, 43, 5000);
    robot->moveToPoint(58.5, 43, 5000, {.minSpeed = 70, .earlyExitRange = 3});

    //touches the bar
    arm_target += 90;
    robot->turnToPoint(25, 51, 5000, {.minSpeed = 70, .earlyExitRange = 3});
    robot->moveToPoint(25, 51, 5000, {}, false);
    intake.move(0);
    





}

void soloAWPBlue() {
    //initialize color sort and position (initial start angle must be accounted for)
    color_sorting_enabled = true;
    autoSelected = true;
    redSide = BLUE;
    robot->setPose(0, 0, 122.33);

    //puts preload on the stake
    arm_target = 235;
    wait(1000);

    //back out, chaining the movement into the turn
    robot->moveToPoint(-12, 12, 5000, {.forwards = false, .minSpeed = 70, .earlyExitRange = 3});
    
    //reset arm position
    wait(250);
    arm_target = 0;

    //turn to the goal, drive forward, and clamp it
    robot->turnToPoint(-14.5, 37.88, 5000, {.forwards = false});
    robot->moveToPoint(-14.5, 37.88, 5000, {.forwards = false, .maxSpeed = 70});
    robot->waitUntil(26);
    mogo.set_value(true);

    //wait before moving the intake so hooks don't catch
    wait(250);
    intake.move(127);

    //intake close ring onto mobile goal
    robot->moveToPoint(-34, 33.88, 1000);

    //chain movements to get to the ring stack 
    robot->moveToPoint(-15, 13, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    robot->turnToPoint(20, 13, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    robot->moveToPoint(7, 13, 1000, {.minSpeed = 40, .earlyExitRange = 2});
    robot->waitUntil(4);

    //deposits the mobile goal, and travels to ring
    mogo.set_value(false);
    robot->moveToPoint(9, 13, 1000, {}, false);
    wait(500);
    robot->moveToPoint(32, 13, 1000, {});
    
    //intakes ring, but keeps it in the intake
    pros::Task intStop([&] {
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 5000) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });

    //clamps the mobile goal
    robot->turnToPoint(34.5, 42, 5000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 1});
    robot->moveToPoint(34.5, 42, 5000, {.forwards = false, .maxSpeed = 80});
    robot->waitUntil(29);
    mogo.set_value(true);
    wait(250);

    //finishes intaking the ring, grabs another
    intake.move(127);
    wait(250);
    robot->turnToPoint(58.5, 43, 5000);
    robot->moveToPoint(58.5, 43, 5000, {.minSpeed = 70, .earlyExitRange = 3});

    //touches the bar
    arm_target += 70;
    robot->turnToPoint(25, 51, 5000, {.minSpeed = 70, .earlyExitRange = 3});
    robot->moveToPoint(25, 51, 5000, {}, false);
    intake.move(0);
    
}

// void skills(){
//     //initialize position and color sort
//     color_sorting_enabled = true;
//     redSide = RED;
//     autoSelected = true;
//     pros::lcd::print(4, "skills");
//     robot->setPose(0, 0, 0);

//     //score alliance stake
//     intake.move(127);
//     wait(500);
//     intake.move(0);

//     //move away from alliance stake and turn towards left goal
//     robot->moveToPoint(0, 14.09, 2500);
//     robot->turnToPoint(-25.979, 14.5, 1000, {.forwards = false});

//     //move into the left goal, slowing down as the robot gets closer to the target
//     robot->moveToPoint(-25.979, 14.5, 2000, {
//         .forwards = false, 
//         .minSpeed = 30, 
//         .earlyExitRange = 20});
//     robot->moveToPoint(-25.979, 14.5, 2000, {
//         .forwards = false, 
//         .maxSpeed = 50});
//     robot->waitUntil(11);
//     mogo.set_value(true);


//     //all the next movements are chained to increase time efficiency

//     //turn to the first ring and begin intaking
//     robot->turnToPoint(-24.155, 37.383, 1000, {.minSpeed = 70, .earlyExitRange = 3});
//     intake.move(127);
//     //move to first ring
//     robot->moveToPoint(-24.155, 37.383, 2000, {.minSpeed = 40, .earlyExitRange = 3});

//     //curve towards second ring in two motions
//     //first motion is offset to make sure we do not contact the ladder
//     robot->moveToPoint(-49.155, 53.383, 2000, {.minSpeed = 40, .earlyExitRange = 6});
//     //second motion ends at the ring
//     robot->moveToPoint(-60.388, 64.25, 2000, {});
//     //add back motion chaining if bad ^ 

//     robot->moveToPoint(-55.735, 64.023, 2000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 3});
//     //turn and move to 3rd ring
//     robot->turnToPoint(-51.735, 47.023, 1000, {.maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 3});
//     robot->moveToPoint(-52.735, 46.023, 2000, {.maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 3});
//     //move to 4th and 5th rings while slowing down for consistenct
//     robot->turnToPoint(-52.31, 19, 1000, { .minSpeed = 70, .earlyExitRange = 3});
//     robot->moveToPoint(-52.31, 19, 1000, {.maxSpeed = 55, .minSpeed = 70, .earlyExitRange = 3});
//     robot->moveToPoint(-48.31, -1, 3000, {.maxSpeed = 50});

//     //turn slowly to make sure ring doesn't fly off and grab 6th ring
//     robot->turnToPoint(-67.675, 13.842, 1000, {.maxSpeed = 62, .minSpeed = 40, .earlyExitRange = 3});
//     robot->moveToPoint(-67.675, 11.842, 2000, {.minSpeed = 70, .earlyExitRange = 3});

//     //drop goal in corner and outtake to make sure goal does not get stuck
//     robot->moveToPoint(-67.289, -4.889, 2000, {.forwards = false, .maxSpeed = 70}, false);
//     mogo.set_value(false);
//     intake.move(-127);
//     robot->moveToPoint(-50, 13, 5000);

//     //grab second goal and slow down as we come closer to it
//     robot->turnToPoint(17.979, 13, 2000, {.forwards = false});
//     intake.move(0);
//     robot->moveToPoint(17.979, 13, 2000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 30});
//     robot->moveToPoint(17.979, 13, 2000, {.forwards = false, .maxSpeed = 30, .minSpeed = 30});
//     robot->waitUntil(89);
//     mogo.set_value(true);

//     //grab 1st ring
//     robot->turnToPoint(21.155, 37.383, 1000, {.minSpeed = 70, .earlyExitRange = 3});
//     robot->moveToPoint(21.155, 37.383, 2000, {});
//     intake.move(127);

//     //go to wallstake while intaking ring
//     // robot->turnToPoint(45.375, 64.563, 700, {.minSpeed = 30, .earlyExitRange = 1});
//     // robot->moveToPoint(45.375, 64.563, 1000, {}, false);

//     robot->turnToPoint(53.375, 61.563, 700, {.minSpeed = 30, .earlyExitRange = 1});
//     robot->moveToPoint(53.375, 61.563, 1000, {}, false);

//     //set arm height
//     arm_target = LOAD_ARM_POS;

//     // drive next to wallstake
//     robot->turnToHeading(90, 1000, {});
//     robot->moveToPoint(64.864, 64.588, 1000, {});
//     // spam intake so ring goes farther back into arm
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     robot->waitUntilDone();

//     // align with wallstake using aligner
//     robot->moveToPoint(70.864, 64.588, 1000, {.maxSpeed = 70});

//     // wait(500);

//     // disable pid to stop conflicts
//     arm_pid_enabled = false;

//     //outtake slowly to stop hooks from jamming rings
//     // intake.move(-40);
//     // int time = 0; //initialize timeout in case of jam

//     //tap again a bunch (test)
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(127);
//     wait(100);
//     intake.move(0);
//     wait(100);
//     intake.move(-40);
//     int time = 0;

//     robot->waitUntilDone();

//     // robot->moveToPoint(70.864, 64.588, 1000, {.maxSpeed = 70});
//     //move lady brown to scoring position
//     while ((176.5 - (ladyBrownRotation.get_angle() / 100.0) > 2) && time < 750) {
//         ladyBrown.move(127);
//         time += 10;
//         wait(10);
//     }
//     ladyBrown.move(0);

//     //let ring settle
//     wait(500);

//     //move away from wallstake
//     robot->moveToPoint(50, 60, 700, {.forwards = false}, false);

//     //reactivate arm pid and set arm down
//     arm_pid_enabled = true;
//     arm_target = LOAD_ARM_POS + 40;
    
//     //intake 2nd ring
//     robot->moveToPoint(47.31, 34.023, 1000, {.minSpeed = 70, .earlyExitRange = 3});
//     intake.move(127);

//     //intake 3rd and 4th rings
//     robot->turnToPoint(47.31, -2.83, 1000, {}, false);
//     robot->moveToPoint(45.31, 13.5, 1000, {.minSpeed = 40, .earlyExitRange = 3});
//     robot->moveToPoint(48.31, 0, 3000, {.maxSpeed = 60});

//     //turn to the 5th ring clockwise so it does not get knocked over
//     robot->turnToPoint(62.675, 18.842, 1000, {
//         .direction = lemlib::AngularDirection::CW_CLOCKWISE,
//         .minSpeed = 70, 
//         .earlyExitRange = 3});
//     robot->moveToPoint(62.675, 18.842, 2000, {.minSpeed = 70, .earlyExitRange = 3});

//     //drop goal into corner and outtake to prevent jam
//     robot->moveToPoint(72.289, -4.889, 2000, {.forwards = false}, false);
//     mogo.set_value(false);
//     wait(200);
//     intake.move(-127);

//     //move to the ring next to the ladder and load it in to our intake
//     robot->moveToPoint(44.58, 90.693, 2000, {.maxSpeed = 100});
//     intake.move(127);
//     pros::Task intStop([&] {
//         //set timeout in case ring does not enter correctly
//         int time = 0;
//         while (intakeDistance.get_distance() > 90 && time < 2750) {
//             intake.move(127);
//             pros::delay(10);
//             time += 10;
//         }
//         intake.move(0);
//     });

//     //end next to the goal
//     robot->turnToPoint(7, 106.698, 750, {.forwards = false});
//     robot->moveToPoint(7, 106.698, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3});

//     //chain movements into grabbing the goal
//     robot->turnToPoint(-16, 108.198, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3});
//     robot->moveToPoint(-16, 108.198, 1000, {.forwards = false, .maxSpeed = 50});
//     robot->waitUntil(16.5);
//     mogo.set_value(true);

//     //intake loaded ring
//     intake.move(127);

//     //turn to rings on other side of ladder and intake both
//     robot->turnToPoint(-33, 79, 1000, {.minSpeed = 40, .earlyExitRange = 3});
//     robot->moveToPoint(-33, 79, 1000, {.minSpeed = 40, .earlyExitRange = 3});
//     robot->turnToPoint(-67, 79, 1000, {.minSpeed = 40, .earlyExitRange = 3});
//     robot->moveToPoint(-67, 79, 1000, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 2});

//     //intake the red rings below the blue rings in a zigzag pattern as to not accidentally intake blue rings
//     //path.jerryio coordinates did not translate perfectly requiring additional offsets
//     robot->turnToPoint(-75.525, 98.123, 750, {.minSpeed = 40, .earlyExitRange = 1});
//     robot->moveToPoint(-75.525, 98.123, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 1});

//     //reverses
//     robot->moveToPoint(-83.525, 97.123, 1000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});

//     robot->turnToPoint(-57.31, 102.123, 1000, {.minSpeed = 40, .earlyExitRange = 2});
//     robot->moveToPoint(-57.31, 102.123, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 2});
//     robot->turnToPoint(-64.31, 120.351, 1000, {.minSpeed = 40, .earlyExitRange = 2});
//     robot->moveToPoint(-64.31, 120.351, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 1});

//     //drop goal off in the corner
//     robot->turnToPoint(-86.564, 127.351 - 6, 750, 
//             {.forwards = false, .minSpeed = 40, .earlyExitRange = 1}, false);
//      robot->moveToPoint(-80.564, 127.351 - 12, 750, 
//             {.forwards = false, .minSpeed = 40, .earlyExitRange = 1}, false);
//     robot->moveToPoint(-86.564, 127.351- 6, 1000, 
//             {.forwards = false, .maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 1});
//     mogo.set_value(false);

//     //chain movements to go around first blue mobile goal and shove the second one into the corner to avoid SG6 violation
//     robot->moveToPoint(-29.044 - 13, 110.998 - 6, 1000, {.minSpeed = 40, .earlyExitRange = 2});
//     robot->moveToPoint(-4.601 - 13, 104.96 + 6, 1000, {.minSpeed = 40, .earlyExitRange = 2});
//     robot->moveToPoint(34.22 - 13, 124.226 + 18, 1000, {.minSpeed = 30, .earlyExitRange = 2});
//     robot->moveToPoint(62.96 - 13, 127.02 + 10, 1000);

//     //robot goes to hang
//     robot->turnToPoint(-12, 56, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2});
//     intake.move(0);
//     arm_target = 200;
//     robot->moveToPoint(-12, 56, 1000, {.forwards = false, .maxSpeed = 105});

//     odomRetract.set_value(true);
// }

void skills(){
    //initialize position and color sort
    color_sorting_enabled = true;
    redSide = RED;
    autoSelected = true;
    pros::lcd::print(4, "skills");
    robot->setPose(0, 0, 0);

    //score alliance stake
    intake.move(127);
    wait(500);
    intake.move(0);

    //move away from alliance stake and turn towards left goal
    robot->moveToPoint(0, 14.09, 2500);
    robot->turnToPoint(-25.979, 14.5, 1000, {.forwards = false});

    //move into the left goal, slowing down as the robot gets closer to the target
    robot->moveToPoint(-25.979, 14.5, 2000, {
        .forwards = false, 
        .minSpeed = 30, 
        .earlyExitRange = 20});
    robot->moveToPoint(-25.979, 14.5, 2000, {
        .forwards = false, 
        .maxSpeed = 50});
    robot->waitUntil(11);
    mogo.set_value(true);


    //all the next movements are chained to increase time efficiency

    //turn to the first ring and begin intaking
    robot->turnToPoint(-24.155, 37.383, 1000, {.minSpeed = 70, .earlyExitRange = 3});
    intake.move(127);
    //move to first ring
    robot->moveToPoint(-24.155, 37.383, 2000, {.minSpeed = 40, .earlyExitRange = 3});

    //curve towards second ring in two motions
    //first motion is offset to make sure we do not contact the ladder
    robot->moveToPoint(-49.155, 56.383, 2000, {.minSpeed = 40, .earlyExitRange = 3});
    //second motion ends at the ring

    robot->moveToPoint(-60.388, 64.25, 2000, {});
    //add back motion chaining if bad ^ 

    arm_target = LOAD_ARM_POS;

    // drive next to wallstake
    robot->turnToHeading(-90, 1000, {});
    robot->moveToPoint(-64.864, 64.588, 1000, {}, false);
    // spam intake so ring goes farther back into arm
    pros::Task spam([&] {
        intake.move(127);
        wait(100);
        intake.move(0);
        wait(100);
        intake.move(127);
        wait(100);
        intake.move(0);
        wait(100);
        intake.move(127);
        wait(100);
        intake.move(0);
        wait(100);
        intake.move(127);
        wait(100);
        intake.move(0);
    });
    robot->waitUntilDone();

    // align with wallstake using aligner
    robot->moveToPoint(-72.864, 64.588, 1000, {.maxSpeed = 70});

    // wait(500);

    //tap again a bunch (test)
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    robot->waitUntilDone();

     // disable pid to stop conflicts
    arm_pid_enabled = false;

    //outtake slowly to stop hooks from jamming rings
    int time = 0; //initialize timeout in case of jam
    intake.move(-40);

    //move lady brown to scoring position
    while ((176.5 - (ladyBrownRotation.get_angle() / 100.0) > 2) && time < 750) {
        ladyBrown.move(127);
        time += 10;
        wait(10);
    }
    ladyBrown.move(0);

    //let ring settle
    wait(500);

    //reactivate arm pid and set arm down
    robot->moveToPoint(-52.735, 64.023, 2000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 3}, false);
    arm_pid_enabled = true;
    arm_target = LOAD_ARM_POS + 40;

    intake.move(127);
    //turn and move to 3rd ring
    robot->turnToPoint(-51.735, 47.023, 1000, {.maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 3});
    robot->moveToPoint(-52.735, 46.023, 2000, {.maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 3});
    //move to 4th and 5th rings while slowing down for consistenct
    robot->turnToPoint(-52.31, 19, 1000, { .minSpeed = 70, .earlyExitRange = 3});
    robot->moveToPoint(-52.31, 19, 1000, {.maxSpeed = 55, .minSpeed = 70, .earlyExitRange = 3});
    robot->moveToPoint(-48.31, -1, 3000, {.maxSpeed = 50});

    //turn slowly to make sure ring doesn't fly off and grab 6th ring
    robot->turnToPoint(-67.675, 13.842, 1000, {.maxSpeed = 62, .minSpeed = 40, .earlyExitRange = 3});
    robot->moveToPoint(-67.675, 11.842, 2000, {.minSpeed = 70, .earlyExitRange = 3});

    //drop goal in corner and outtake to make sure goal does not get stuck
    robot->moveToPoint(-67.289, -4.889, 2000, {.forwards = false, .maxSpeed = 70}, false);
    mogo.set_value(false);
    intake.move(-127);
    // arm_target = LOAD_ARM_POS;
    robot->moveToPoint(-50, 13, 5000);

    //grab second goal and slow down as we come closer to it
    robot->turnToPoint(17.979, 15, 2000, {.forwards = false});
    robot->moveToPoint(17.979, 15, 2000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 30});
    intake.move(0);
    // arm_target = BASE_ARM_POS;
    robot->moveToPoint(17.979, 15, 2000, {.forwards = false, .maxSpeed = 30, .minSpeed = 30});
    robot->waitUntil(89);
    mogo.set_value(true);

    //grab 1st ring
    robot->turnToPoint(21.155, 37.383, 1000, {.minSpeed = 70, .earlyExitRange = 3});
    robot->moveToPoint(21.155, 37.383, 2000, {});
    intake.move(127);
    robot->waitUntilDone();
    robot->setPose(21.155, 37.383, robot->getPose().theta);

    //go to wallstake while intaking ring
    // robot->turnToPoint(45.375, 64.563, 700, {.minSpeed = 30, .earlyExitRange = 1});
    // robot->moveToPoint(45.375, 64.563, 1000, {}, false);

    robot->turnToPoint(51.375, 61.563, 700, {.minSpeed = 30, .earlyExitRange = 1});
    robot->moveToPoint(51.375, 61.563, 1000, {}, false);

    //set arm height
    arm_target = LOAD_ARM_POS;

    // drive next to wallstake
    robot->turnToHeading(90, 1000, {});
    robot->moveToPoint(64.864, 62.588, 1000, {});
    wait(500);
    // spam intake so ring goes farther back into arm
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    robot->waitUntilDone();

    // align with wallstake using aligner
    robot->moveToPoint(70.864, 62.588, 1000, {.maxSpeed = 70});

    // wait(500);

    // disable pid to stop conflicts
    arm_pid_enabled = false;

    //outtake slowly to stop hooks from jamming rings
    // intake.move(-40);
    // int time = 0; //initialize timeout in case of jam

    //tap again a bunch (test)
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(127);
    wait(100);
    intake.move(0);
    wait(100);
    intake.move(-40);
    time = 0;

    robot->waitUntilDone();

    // robot->moveToPoint(70.864, 64.588, 1000, {.maxSpeed = 70});
    //move lady brown to scoring position
    while ((176.5 - (ladyBrownRotation.get_angle() / 100.0) > 2) && time < 750) {
        ladyBrown.move(127);
        time += 10;
        wait(10);
    }
    ladyBrown.move(0);

    //let ring settle
    wait(500);

    //move away from wallstake
    robot->moveToPoint(50, 60, 700, {.forwards = false}, false);

    //reactivate arm pid and set arm down
    arm_pid_enabled = true;
    arm_target = LOAD_ARM_POS + 40;
    
    //intake 2nd ring
    robot->moveToPoint(47.31, 34.023, 1000, {.minSpeed = 70, .earlyExitRange = 3});
    intake.move(127);

    //intake 3rd and 4th rings
    robot->turnToPoint(47.31, -2.83, 1000, {}, false);
    robot->moveToPoint(45.31, 13.5, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    robot->moveToPoint(48.31, 0, 3000, {.maxSpeed = 60});

    //turn to the 5th ring clockwise so it does not get knocked over
    robot->turnToPoint(62.675, 18.842, 1000, {
        .direction = lemlib::AngularDirection::CW_CLOCKWISE,
        .minSpeed = 70, 
        .earlyExitRange = 3});
    intake.move(0);
    robot->waitUntilDone();
    intake.move(127);
    robot->moveToPoint(62.675, 18.842, 2000, {.minSpeed = 70, .earlyExitRange = 3});

    //drop goal into corner and outtake to prevent jam
    robot->moveToPoint(67.289, -4.889, 2000, {.forwards = false}, false);
    mogo.set_value(false);
    wait(200);
    intake.move(-127);

    //move to the ring next to the ladder and load it in to our intake
    robot->moveToPoint(41.58, 90.693, 2000, {.maxSpeed = 100});
    intake.move(127);
    pros::Task intStop([&] {
        //set timeout in case ring does not enter correctly
        int time = 0;
        while (intakeDistance.get_distance() > 90 && time < 2750) {
            intake.move(127);
            pros::delay(10);
            time += 10;
        }
        intake.move(0);
    });

    //end next to the goal
    robot->turnToPoint(7, 106.698, 750, {.forwards = false});
    robot->moveToPoint(7, 106.698, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3});

    //chain movements into grabbing the goal
    robot->turnToPoint(-16, 108.198, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3});
    robot->moveToPoint(-16, 108.198, 1000, {.forwards = false, .maxSpeed = 50});
    robot->waitUntil(16.5);
    mogo.set_value(true);

    //intake loaded ring
    intake.move(127);

    //turn to rings on other side of ladder and intake both
    robot->turnToPoint(-33, 79, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    robot->moveToPoint(-33, 79, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    robot->turnToPoint(-67, 79, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    robot->moveToPoint(-67, 79, 1000, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 2});

    //intake the red rings below the blue rings in a zigzag pattern as to not accidentally intake blue rings
    //path.jerryio coordinates did not translate perfectly requiring additional offsets
    robot->turnToPoint(-75.525, 98.123, 750, {.minSpeed = 40, .earlyExitRange = 1});
    robot->moveToPoint(-75.525, 98.123, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 1});

    //reverses
    robot->moveToPoint(-83.525, 97.123, 1000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2});

    robot->turnToPoint(-57.31, 102.123, 1000, {.minSpeed = 40, .earlyExitRange = 2});
    robot->moveToPoint(-57.31, 102.123, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 2});
    robot->turnToPoint(-64.31, 120.351, 1000, {.minSpeed = 40, .earlyExitRange = 2});
    robot->moveToPoint(-64.31, 120.351, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 1});
    robot->turnToPoint(-86, 120, 750, {});
    doinker.set_value(true);
    //drop goal off in the corner
    robot->turnToPoint(-86.564, 127.351 - 6, 750, 
            {.forwards = false, .minSpeed = 40, .earlyExitRange = 1}, false);
    doinker.set_value(false);
    robot->moveToPoint(-80.564, 127.351 - 12, 750, 
            {.forwards = false, .minSpeed = 40, .earlyExitRange = 1}, false);
    robot->moveToPoint(-86.564, 127.351- 6, 1000, 
            {.forwards = false, .maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 1});
    mogo.set_value(false);

    //chain movements to go around first blue mobile goal and shove the second one into the corner to avoid SG6 violation
    intake.move(-127);
    robot->moveToPoint(-29.044 - 13, 110.998 - 6, 1000, {.minSpeed = 30, .earlyExitRange = 2});
    robot->moveToPoint(-4.601 - 13, 104.96, 1000, {.minSpeed = 40, .earlyExitRange = 2});
    robot->moveToPoint(34.22 - 13, 124.226 + 10, 1000, {.minSpeed = 30, .earlyExitRange = 2});
    robot->moveToPoint(62.96 - 13, 127.02 + 6, 1000);

    //robot goes to hang
    robot->turnToPoint(-12, 56, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2});
    intake.move(0);
    robot->moveToPoint(-12, 56, 1000, {.forwards = false, .maxSpeed = 115});
    wait(200);
    arm_target = 200;
    odomRetract.set_value(true);
}