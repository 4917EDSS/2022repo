// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignToVisionCmd.h"



AlignToVisionCmd::AlignToVisionCmd(DrivetrainSub *drivetrainSub, VisionSub *visionSub) {
  AddRequirements({drivetrainSub,visionSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_visionSubPtr = visionSub;
  
}

// Called when the command is initially scheduled.
void AlignToVisionCmd::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void AlignToVisionCmd::Execute() {
  double currentAngle = m_visionSubPtr->getHorizontalAngle();
  if(fabs(currentAngle) < 5) {
      currentAngle = 5*fabs(currentAngle)/currentAngle;
    }
  if(currentAngle != 0){
    m_drivetrainSubPtr->arcadeDrive(0, currentAngle/20);
  }
}

// Called once the command ends or is interrupted.
void AlignToVisionCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0, 0);
}

// Returns true when the command should end.
bool AlignToVisionCmd::IsFinished() {
  double currentAngle = m_visionSubPtr->getHorizontalAngle();
  if(fabs(currentAngle) < .5 && (fabs(m_drivetrainSubPtr->getLeftVelocity()) + fabs(m_drivetrainSubPtr->getRightVelocity())) < .01){
    return true;
  }
  else{
    return false;
  }
}