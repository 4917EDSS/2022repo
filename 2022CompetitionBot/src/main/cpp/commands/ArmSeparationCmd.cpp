// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmSeparationCmd.h"

ArmSeparationCmd::ArmSeparationCmd(ClimberSub * armSepartion, bool armSeparationDirection) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({armSepartion});
  m_armSeparationPtr = armSepartion;
  m_armSeparationDirection = armSeparationDirection;
}

// Called when the command is initially scheduled.
void ArmSeparationCmd::Initialize() {
  if (m_armSeparationDirection)
    m_armSeparationPtr->raiseArmSeparation();
  else
    m_armSeparationPtr->lowerArmSeparation();

}

// Called repeatedly when this Command is scheduled to run
void ArmSeparationCmd::Execute() {}

// Called once the command ends or is interrupted.
void ArmSeparationCmd::End(bool interrupted) {
  m_armSeparationPtr->lowerArmSeparation();
}

// Returns true when the command should end.
bool ArmSeparationCmd::IsFinished() {
  return false;
}
