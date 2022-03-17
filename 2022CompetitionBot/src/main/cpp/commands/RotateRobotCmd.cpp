// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateRobotCmd.h"
#include "subsystems/DrivetrainSub.h"

constexpr double kMinPower = 0.15;
constexpr double kTolerance = 0.15;

RotateRobotCmd::RotateRobotCmd(DrivetrainSub *drivetrainSub, double angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_angle = angle;
}

// Called when the command is initially scheduled.
void RotateRobotCmd::Initialize() {
  m_drivetrainSubPtr->zeroHeading();
}

// Called repeatedly when this Command is scheduled to run
void RotateRobotCmd::Execute() {
  double power = 1;
  rotationRemaning = m_angle-m_drivetrainSubPtr->getHeading();
  double dir = (rotationRemaning < 0) ? -1: 1;
  rotationRemaning = fabs(rotationRemaning);
  if (rotationRemaning <= 15){ power = rotationRemaning/15; }
  if (power <= kMinPower){ power = kMinPower; }

  m_drivetrainSubPtr->arcadeDrive(0, power*dir);
}

// Called once the command ends or is interrupted.
void RotateRobotCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0, 0);
}

// Returns true when the command should end.
bool RotateRobotCmd::IsFinished() {
  if ((rotationRemaning <= kTolerance) && (fabs(m_drivetrainSubPtr->getVelocity())<=0.1)){
    return true;
  }
  return false;
}
