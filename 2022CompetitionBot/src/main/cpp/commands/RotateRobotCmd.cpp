// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateRobotCmd.h"
#include "subsystems/DrivetrainSub.h"

constexpr double kMinPower = 0.3;
constexpr double kMaxPower = 1;
constexpr double kTolerance = 1;//degrees

RotateRobotCmd::RotateRobotCmd(DrivetrainSub *drivetrainSub, double angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_angle = angle;
}

// Called when the command is initially scheduled.
void RotateRobotCmd::Initialize() {
  m_drivetrainSubPtr->shiftDown();
  m_drivetrainSubPtr->zeroHeading();
}

// Called repeatedly when this Command is scheduled to run
void RotateRobotCmd::Execute() {
  double power = 1;
  rotationRemaining = m_angle-m_drivetrainSubPtr->getHeading();
  double dir = (rotationRemaining < 0) ? -1: 1;
  rotationRemaining = fabs(rotationRemaining);
  if (rotationRemaining <= 40){ power = ((rotationRemaining/40)*(kMaxPower-kMinPower))+kMinPower; }
  if (rotationRemaining <= kTolerance) { power = 0; }

  m_drivetrainSubPtr->arcadeDrive(0, power*dir);
}

// Called once the command ends or is interrupted.
void RotateRobotCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0, 0);
}

// Returns true when the command should end.
bool RotateRobotCmd::IsFinished() {
  if ((rotationRemaining <= kTolerance) && (fabs(m_drivetrainSubPtr->getTurnRate())<=5)){
    return true;
  }
  return false;
}
