// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveStraightCmd.h"
#include "subsystems/DrivetrainSub.h"

constexpr double kRotateAdjustment = 0.03;

DriveStraightCmd::DriveStraightCmd(DrivetrainSub *drivetrainSub, double driveStraightDistance) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_driveStraightDistance = driveStraightDistance;
}

// Called when the command is initially scheduled.
void DriveStraightCmd::Initialize() {
  m_drivetrainSubPtr->zeroHeading();
  m_drivetrainSubPtr->zeroDrivetrainEncoders();
  m_drivetrainSubPtr->shiftDown();
  power = 0.5;
  if (m_driveStraightDistance < 0) {
    power *= -1;
  }
}

// Called repeatedly when this Command is scheduled to run
void DriveStraightCmd::Execute() {
  double rotatePwr = m_drivetrainSubPtr->getHeading()*kRotateAdjustment;
  m_drivetrainSubPtr->arcadeDrive(power, -rotatePwr);
}

// Called once the command ends or is interrupted.
void DriveStraightCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0,0);
}

// Returns true when the command should end.
bool DriveStraightCmd::IsFinished() {
  if (power >= 0) {
    if ((m_drivetrainSubPtr->getLeftEncoderDistanceM()>m_driveStraightDistance) && (m_drivetrainSubPtr->getRightEncoderDistanceM()>m_driveStraightDistance)) {
      return true;
    }
  } else {
     if ((m_drivetrainSubPtr->getLeftEncoderDistanceM()<m_driveStraightDistance) && (m_drivetrainSubPtr->getRightEncoderDistanceM()<m_driveStraightDistance)) {
      return true;
    }
  }
  return false;
}
