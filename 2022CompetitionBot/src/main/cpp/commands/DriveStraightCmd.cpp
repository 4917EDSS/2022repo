// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveStraightCmd.h"

DriveStraightCmd::DriveStraightCmd(DrivetrainSub *drivetrainSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
}

// Called when the command is initially scheduled.
void DriveStraightCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveStraightCmd::Execute() {}

// Called once the command ends or is interrupted.
void DriveStraightCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveStraightCmd::IsFinished() {
  return false;
}
