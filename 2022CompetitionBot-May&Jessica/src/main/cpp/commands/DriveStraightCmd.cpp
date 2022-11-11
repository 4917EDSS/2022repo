// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveStraightCmd.h"
#include "subsystems/DrivetrainSub.h"

constexpr double kRotateAdjustment = 0.045;
constexpr double kMinPower = 0.15;
constexpr double kTolerance = 0.03;

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
}

// Called repeatedly when this Command is scheduled to run
void DriveStraightCmd::Execute() {
  double rotatePwr = m_drivetrainSubPtr->getHeading()*kRotateAdjustment;
  double power = 1;
  distanceRemaining = m_driveStraightDistance-m_drivetrainSubPtr->getEncoderDistanceM();
  double dir = (distanceRemaining < 0) ? -1 : 1;
  distanceRemaining = fabs(distanceRemaining);

  if(distanceRemaining <= 0.4) { 
    power = (distanceRemaining / 0.4) * (1 - kMinPower) + kMinPower;
  }
  if(distanceRemaining <= kTolerance)
  {
    power = 0;
  }

  m_drivetrainSubPtr->arcadeDrive(power * dir, -rotatePwr);
}

// Called once the command ends or is interrupted.
void DriveStraightCmd::End(bool interrupted) {
  m_drivetrainSubPtr->arcadeDrive(0,0);
}
 
// Returns true when the command should end.
bool DriveStraightCmd::IsFinished() {
  if((distanceRemaining <= kTolerance) && (fabs(m_drivetrainSubPtr->getVelocity()) <= 0.1)) {
     return true; 
  }
  return false;
}