// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveStraightCmd.h"
#include "subsystems/DrivetrainSub.h"

DriveStraightCmd::DriveStraightCmd(DrivetrainSub *drivetrainSub, double driveStraightDistance) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_driveStraightDistance = driveStraightDistance;
}

// Called when the command is initially scheduled.
void DriveStraightCmd::Initialize() {
  m_drivetrainSubPtr->shiftDown();
}

// Called repeatedly when this Command is scheduled to run
void DriveStraightCmd::Execute() {
 m_drivetrainSubPtr->arcadeDrive(0.5, 0);//temporarily testing with 50% power
}

// Called once the command ends or is interrupted.
void DriveStraightCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0,0);
}

// Returns true when the command should end.
bool DriveStraightCmd::IsFinished() {
  if ((m_drivetrainSubPtr->getLeftEncoderDistanceM()>m_driveStraightDistance) && (m_drivetrainSubPtr->getLeftEncoderDistanceM()>m_driveStraightDistance)) {
    return true;
  }
  return false;
}
