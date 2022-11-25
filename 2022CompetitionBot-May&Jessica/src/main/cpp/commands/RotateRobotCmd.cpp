// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateRobotCmd.h"
#include "subsystems/DrivetrainSub.h"

#include <frc/RobotController.h>

constexpr double kMinPower = 0.2; //
constexpr double kMaxPower = 0.8; // changed from 1, 4/4/22
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
  m_startTime = frc::RobotController::GetFPGATime();
}

// Called repeatedly when this Command is scheduled to run
void RotateRobotCmd::Execute() {
  double power = 0.8; //
  rotationRemaining = m_angle-m_drivetrainSubPtr->getHeading();
  double dir = (rotationRemaining < 0) ? -1: 1; 
  rotationRemaining = fabs(rotationRemaining);
  if(rotationRemaining <= 40)
  {
    power = ((rotationRemaining/40)*(kMaxPower-kMinPower))+kMinPower; 
  }
  if(rotationRemaining <= kTolerance) { 
    power = 0; 
  }

  m_drivetrainSubPtr->arcadeDrive(0, power * dir);
}

// Called once the command ends or is interrupted.
void RotateRobotCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0, 0);
}

// Returns true when the command should end.
bool RotateRobotCmd::IsFinished() {
  if((rotationRemaining <= kTolerance) && (fabs(m_drivetrainSubPtr->getTurnRate()) <= 0.1)) {
    return true;
  }
  if((frc::RobotController::GetFPGATime() - m_startTime) > 3000000) {
    return true;
  }
  return false;
}
