// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RaiseClimberArmCmd.h"

constexpr double kClimberArmPower = 1.0;
constexpr int kClimberArmMaxHeight = 180000;

RaiseClimberArmCmd::RaiseClimberArmCmd(ClimberSub* climberSub) {
  AddRequirements({climberSub});
  m_climberSubPtr = climberSub;
}

// Called when the command is initially scheduled.
void RaiseClimberArmCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RaiseClimberArmCmd::Execute() {
  m_climberSubPtr->setClimberArmPower(kClimberArmPower);
}

// Called once the command ends or is interrupted.
void RaiseClimberArmCmd::End(bool interrupted) {
  m_climberSubPtr->setClimberArmPower(0.);
}

// Returns true when the command should end.
bool RaiseClimberArmCmd::IsFinished() {
  if (m_climberSubPtr->getClimberEncoder() > kClimberArmMaxHeight) {
    return true;
  }
  else {
    return false;
  }
}
