// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShiftLowCmd.h"

ShiftLowCmd::ShiftLowCmd(DrivetrainSub *drivetrainSub) {
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
}

// Called when the command is initially scheduled.
void ShiftLowCmd::Initialize() {
  m_drivetrainSubPtr->setIsAutoShift(false);
  m_drivetrainSubPtr->shiftDown();
}

// Called repeatedly when this Command is scheduled to run
void ShiftLowCmd::Execute() {}

// Called once the command ends or is interrupted.
void ShiftLowCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ShiftLowCmd::IsFinished() {
  return true;
}
