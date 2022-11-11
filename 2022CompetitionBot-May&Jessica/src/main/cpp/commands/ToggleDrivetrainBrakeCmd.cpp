// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ToggleDrivetrainBrakeCmd.h"

ToggleDrivetrainBrakeCmd::ToggleDrivetrainBrakeCmd(DrivetrainSub* drivetrainSub, bool shouldBrake) {
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_shouldBrake = shouldBrake;
}

// Called when the command is initially scheduled.
void ToggleDrivetrainBrakeCmd::Initialize() {
  m_drivetrainSubPtr->setBrakeMode(m_shouldBrake);
}

// Called repeatedly when this Command is scheduled to run
void ToggleDrivetrainBrakeCmd::Execute() {}

// Called once the command ends or is interrupted.
void ToggleDrivetrainBrakeCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ToggleDrivetrainBrakeCmd::IsFinished() {
  return true;
}
