// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSub.h"

VisionSub::VisionSub() {
    init();
}
void VisionSub::init() {
    frc::ShuffleboardTab& visTab = frc::Shuffleboard::GetTab("Vision Data");
    distEntry = visTab.Add("Distance to goal",0.).GetEntry();
}
// This method will be called once per scheduler run
void VisionSub::Periodic() {
    distEntry.SetDouble(VisionSub::estimateDistanceInches());
}

double VisionSub::estimateDistanceInches() { //estimate from camera to goal
    double verticalOffset = VisionSub::getVerticalAngle();

    double angleToGoal = VisionConstants::kMountAngleDegrees+verticalOffset;
    double goalToRadians = angleToGoal*(3.14159/180.0);

    double distToGoal = (VisionConstants::kGoalHeightInches-VisionConstants::kLensHeightInches)/tan(goalToRadians);
    return distToGoal;
}