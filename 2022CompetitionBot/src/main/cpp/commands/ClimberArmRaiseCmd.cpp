// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberArmRaiseCmd.h"

constexpr double kClimberArmPower = 1.0;
constexpr int kClimberArmMaxHeight = 206000;

ClimberArmRaiseCmd::ClimberArmRaiseCmd(ClimberSub* climberSub, double targetHeightPercentage) {
  AddRequirements({climberSub});
  m_climberSubPtr = climberSub; 
  m_targetHeightPercentage = targetHeightPercentage;
}

// Called when the command is initially scheduled.
void ClimberArmRaiseCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ClimberArmRaiseCmd::Execute() {
  m_climberSubPtr->setClimberArmPower(kClimberArmPower);
}

// Called once the command ends or is interrupted.
void ClimberArmRaiseCmd::End(bool interrupted) {
  m_climberSubPtr->setClimberArmPower(0.0);
}

// Returns true when the command should end.
bool ClimberArmRaiseCmd::IsFinished() {
  double targetHeight =  (m_targetHeightPercentage / 100) * (double)kClimberArmMaxHeight;

  if (m_climberSubPtr->getClimberEncoder() > targetHeight) {
    return true;
  }
  else {
    return false;
  }
}
