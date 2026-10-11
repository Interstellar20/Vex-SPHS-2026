#pragma once

void liftSetup();

void liftUp();
void liftDown();
void liftStop();

void toggleMatchLoad();
bool isMatchLoadMoving();
bool moveLiftToPosition(double targetPosition);

double getLiftPosition();
double getNormalLiftPosition();
double getMatchLoadPosition();

void resetLiftPosition();