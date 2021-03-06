// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSub.h"

VisionSub::VisionSub() {
    init();
    frc::ShuffleboardTab& visTab = frc::Shuffleboard::GetTab("Vision Data");
    distEntry = visTab.Add("Distance to goal",0.).GetEntry();
    angleEntry = visTab.Add("Angle to goal",0.).GetEntry();
}

void VisionSub::init() {
    targetVisionPipeline();
}

// This method will be called once per scheduler run
void VisionSub::Periodic() {
    //distEntry.SetDouble(estimateDistanceMeters());
    //angleEntry.SetDouble(getHorizontalAngle());
}

double VisionSub::estimateDistanceMeters() { //estimate from camera to goal
    if (getTargetArea() == 0.0) {
      return 0.0;
    }
    double verticalOffset = getVerticalAngle();

    double angleToGoal = VisionConstants::kMountAngleDegrees+verticalOffset;
    double goalToRadians = angleToGoal*(3.14159/180.0);

    double distToGoal = (VisionConstants::kGoalHeightInches-VisionConstants::kLensHeightInches)/tan(goalToRadians);
    return distToGoal / 39.37;
}

void VisionSub:: targetVisionPipeline() {
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", 1.0);
}

void VisionSub::targetNeutralVisionPipeline() {
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", 0.0);
}
