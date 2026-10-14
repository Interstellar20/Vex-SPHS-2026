#pragma once

void liftSetup();

void liftUp();
void liftDown();
void liftStop();

double getLiftPosition();

double getNormalLiftPosition();
double getMatchLoadPosition();
double getMaxLiftPosition();

bool moveLiftToPosition(double targetPosition);

void toggleMatchLoad();
void updateMatchLoad();
void cancelMatchLoad();

bool isMatchLoadMoving();