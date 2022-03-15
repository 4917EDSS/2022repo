// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateRobotCmd.h"
#include "subsystems/DrivetrainSub.h"

RotateRobotCmd::RotateRobotCmd(DrivetrainSub *drivetrainSub, double angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_angle = angle;
}

// Called when the command is initially scheduled.
void RotateRobotCmd::Initialize() {
  m_drivetrainSubPtr->zeroHeading();
  power = 0.5;
  if (m_angle < m_drivetrainSubPtr->getHeading()) {
    power *= -1;
  }
}

// Called repeatedly when this Command is scheduled to run
void RotateRobotCmd::Execute() {
  m_drivetrainSubPtr->arcadeDrive(0, power);
}

// Called once the command ends or is interrupted.
void RotateRobotCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0, 0);
}

// Returns true when the command should end.
bool RotateRobotCmd::IsFinished() {
  if (power >= 0) {
    if (m_drivetrainSubPtr->getHeading()>m_angle) {
      return true;
    }
  } else {
    if (m_drivetrainSubPtr->getHeading()<m_angle) {
      return true;
    }
  }
  return false;
}
