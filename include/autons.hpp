#pragma once
#include "definitions.hpp"

float wrap180(float angle);
float clamp(float output, float min, float max);

void scoreArm();
void retractArm();

void goalSideRed();
void goalSideBlue();
void ringSideRed();
void ringSideBlue();
void skills();

void safeAutoLeft();
void safeAutoRight();

void testAuton();


