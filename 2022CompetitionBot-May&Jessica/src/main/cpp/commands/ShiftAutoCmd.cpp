// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShiftAutoCmd.h"

ShiftAutoCmd::ShiftAutoCmd(DrivetrainSub *drivetrainSub) {
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShiftAutoCmd::Initialize() {
  m_drivetrainSubPtr->setIsAutoShift(true);
  m_drivetrainSubPtr->autoShift();
}

// Called repeatedly when this Command is scheduled to run
void ShiftAutoCmd::Execute() {}

// Called once the command ends or is interrupted.
void ShiftAutoCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ShiftAutoCmd::IsFinished() {
  return true;
}
