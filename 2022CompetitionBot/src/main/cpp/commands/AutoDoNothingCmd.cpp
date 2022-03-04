// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDoNothingCmd.h"

AutoDoNothingCmd::AutoDoNothingCmd() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoDoNothingCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoDoNothingCmd::Execute() {}

// Called once the command ends or is interrupted.
void AutoDoNothingCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoDoNothingCmd::IsFinished() {
  return false;
}
