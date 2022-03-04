// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ToggleIntakeArmCmd.h"

ToggleIntakeArmCmd::ToggleIntakeArmCmd(IntakeSub* intakeSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intakeSub});
  m_intakeSubPtr = intakeSub;
}

// Called when the command is initially scheduled.
void ToggleIntakeArmCmd::Initialize() {
  m_intakeSubPtr->toggleIntakeArm();
}

// Called repeatedly when this Command is scheduled to run
void ToggleIntakeArmCmd::Execute() {}

// Called once the command ends or is interrupted.
void ToggleIntakeArmCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ToggleIntakeArmCmd::IsFinished() {
  return false;
}
