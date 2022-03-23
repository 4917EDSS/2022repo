// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignToVisionGyroCmd.h"
#include <frc/RobotController.h>
#include <iostream>

constexpr double kMinPower = 0.4;

AlignToVisionGyroCmd::AlignToVisionGyroCmd(DrivetrainSub *drivetrainSub, VisionSub *visionSub) {
  AddRequirements({drivetrainSub,visionSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_visionSubPtr = visionSub;
}

// Called when the command is initially scheduled.
void AlignToVisionGyroCmd::Initialize() {
  m_visionSubPtr->targetVisionPipeline();
  m_startTime = frc::RobotController::GetFPGATime();
  
  m_angle = m_visionSubPtr->getHorizontalAngle(); 
  m_drivetrainSubPtr->zeroHeading(); 
}
//-12 deg   -- 0 deg
// -12-0 = -12
//  -12/20.5 =/ -0.5
//  dir = -1
//  p=0.5
//  a = 12

// Called repeatedly when this Command is scheduled to run
void AlignToVisionGyroCmd::Execute() { //Basically copied from RotateRobotCmd
  if( m_visionSubPtr->getHorizontalAngle() == 0.0) m_angle =  m_visionSubPtr->getHorizontalAngle();
  else {
    m_angle = m_visionSubPtr->getHorizontalAngle(); 
    double angleRemaining = m_angle-m_drivetrainSubPtr->getHeading();
    double rotationPower = (angleRemaining/20.5);
    double dir = (angleRemaining < 0.0) ? -1.0: 1.0;
    rotationPower = fabs(rotationPower);
    angleRemaining = fabs(angleRemaining);
    if(rotationPower < kMinPower && rotationPower > 0){ rotationPower = kMinPower; }

    if(angleRemaining < .5){ //.5 degrees
      m_drivetrainSubPtr->arcadeDrive(0, 0);
    } else {
      m_drivetrainSubPtr->arcadeDrive(0, rotationPower*dir);
    }
  } 
}

// Called once the command ends or is interrupted.
void AlignToVisionGyroCmd::End(bool interrupted) {
  m_visionSubPtr->targetNeutralVisionPipeline(); //Turn off camera
}

// Returns true when the command should end.
bool AlignToVisionGyroCmd::IsFinished() {
  return false;
}
