// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShiftHighCmd.h"

ShiftHighCmd::ShiftHighCmd(DrivetrainSub *drivetrainSub) {
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
}

// Called when the command is initially scheduled.
void ShiftHighCmd::Initialize() {
  m_drivetrainSubPtr->setIsAutoShift(false);
  m_drivetrainSubPtr->shiftUp();
}

// Called repeatedly when this Command is scheduled to run
void ShiftHighCmd::Execute() {}

// Called once the command ends or is interrupted.
void ShiftHighCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ShiftHighCmd::IsFinished() {
  return false;
}
