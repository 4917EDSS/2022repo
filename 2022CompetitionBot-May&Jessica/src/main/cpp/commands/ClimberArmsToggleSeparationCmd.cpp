// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberArmsToggleSeparationCmd.h"

CimberArmsToggleSeparationCmd::CimberArmsToggleSeparationCmd(ClimberSub *climberSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climberSub});
  m_climberSubPtr = climberSub;
}

// Called when the command is initially scheduled.
void CimberArmsToggleSeparationCmd::Initialize() {
  m_climberSubPtr->toggleArmSeparation();
}

// Called repeatedly when this Command is scheduled to run
void CimberArmsToggleSeparationCmd::Execute() {}

// Called once the command ends or is interrupted.
void CimberArmsToggleSeparationCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool CimberArmsToggleSeparationCmd::IsFinished() {
  return true;
}
