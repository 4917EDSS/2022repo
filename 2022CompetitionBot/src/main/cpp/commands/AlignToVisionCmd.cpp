// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignToVisionCmd.h"
#include "commands/RotateRobotCmd.h"
#include <frc/RobotController.h>

constexpr double kMinPower = 0.4;

AlignToVisionCmd::AlignToVisionCmd(DrivetrainSub *drivetrainSub, VisionSub *visionSub) {
  AddRequirements({drivetrainSub,visionSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_visionSubPtr = visionSub;
}

// Called when the command is initially scheduled.
void AlignToVisionCmd::Initialize() {
  m_visionSubPtr->targetVisionPipeline();
  m_startTime = frc::RobotController::GetFPGATime();
}

// Called repeatedly when this Command is scheduled to run
void AlignToVisionCmd::Execute() {
  angleRemaining = m_visionSubPtr->getHorizontalAngle();
  RotateRobotCmd(m_drivetrainSubPtr, angleRemaining);
}

// Called once the command ends or is interrupted.
void AlignToVisionCmd::End(bool interrupted) {
  m_visionSubPtr->targetNeutralVisionPipeline();
  m_drivetrainSubPtr->arcadeDrive(0, 0);
}

// Returns true when the command should end.
bool AlignToVisionCmd::IsFinished() {
  if (m_visionSubPtr->isValidTarget() == 0){
    return true;
  }
  if((frc::RobotController::GetFPGATime() - m_startTime) > 5000000) {
    return true;
  }
  if(fabs(angleRemaining) < .5 && fabs(m_drivetrainSubPtr->getTurnRate()) <= 2) {
    return true;
  }
  else{
    return false;
  }
}