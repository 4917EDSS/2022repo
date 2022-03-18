// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignToVisionCmd.h"
#include <frc/RobotController.h>

constexpr double kMinPower = 0.4;

AlignToVisionCmd::AlignToVisionCmd(DrivetrainSub *drivetrainSub, VisionSub *visionSub) {
  AddRequirements({drivetrainSub,visionSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_visionSubPtr = visionSub;
}

// Called when the command is initially scheduled.
void AlignToVisionCmd::Initialize() {
  m_startTime = frc::RobotController::GetFPGATime();
}

// Called repeatedly when this Command is scheduled to run
void AlignToVisionCmd::Execute() {
  double angleRemaining = m_visionSubPtr->getHorizontalAngle();
  double rotationSpeed = angleRemaining/20;
  int dir = (rotationSpeed < 0) ? -1: 1;
  rotationSpeed = fabs(rotationSpeed);
  angleRemaining = fabs(angleRemaining);
  if(rotationSpeed < kMinPower && rotationSpeed > 0){ rotationSpeed = kMinPower; }
  if(angleRemaining < .5){
    m_drivetrainSubPtr->arcadeDrive(0, 0);
  } else {
    m_drivetrainSubPtr->arcadeDrive(0, rotationSpeed*dir);
  }
}

// Called once the command ends or is interrupted.
void AlignToVisionCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0, 0);
}

// Returns true when the command should end.
bool AlignToVisionCmd::IsFinished() {
  double angleRemaining = fabs(m_visionSubPtr->getHorizontalAngle());
  if (m_visionSubPtr->getTargetArea() == 0){
    return true;
  }
  if((frc::RobotController::GetFPGATime() - m_startTime) > 5000000) {
    return true;
  }
  if(angleRemaining < .5 && fabs(m_drivetrainSubPtr->getTurnRate()) <= 0.3) {
    return true;
  }
  else{
    return false;
  }
}