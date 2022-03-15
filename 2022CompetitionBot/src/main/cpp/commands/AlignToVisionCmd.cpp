// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignToVisionCmd.h"
#include <frc/RobotController.h>



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
  double currentAngle = m_visionSubPtr->getHorizontalAngle();
  double rotationSpeed = currentAngle/20;
  if(rotationSpeed < 0.25 && rotationSpeed > 0){
    rotationSpeed = 0.25;
  } else if(rotationSpeed > -0.25 && rotationSpeed < 0){
    rotationSpeed = -0.25;
  }
  if(fabs(currentAngle) < .5){
    m_drivetrainSubPtr->arcadeDrive(0, 0);
  } else {
    m_drivetrainSubPtr->arcadeDrive(0, rotationSpeed);
  }
}

// Called once the command ends or is interrupted.
void AlignToVisionCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0, 0);
}

// Returns true when the command should end.
bool AlignToVisionCmd::IsFinished() {
  double currentAngle = m_visionSubPtr->getHorizontalAngle();
  if (m_visionSubPtr->getTargetArea() == 0){
    return true;
  }
  if((frc::RobotController::GetFPGATime() - m_startTime) > 5000000) {
    return true;
  }
  if(fabs(currentAngle) < .5 && (fabs(m_drivetrainSubPtr->getLeftVelocity()) + fabs(m_drivetrainSubPtr->getRightVelocity())) < .01){
    return true;
  }
  else{
    return false;
  }
}