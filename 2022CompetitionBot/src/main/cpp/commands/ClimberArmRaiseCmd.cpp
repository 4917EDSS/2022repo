// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberArmRaiseCmd.h"

constexpr double kClimberArmPower = 1.0;
constexpr int kClimberArmMaxHeight = 203000;

ClimberArmRaiseCmd::ClimberArmRaiseCmd(ClimberSub* climberSub, bool isPartial) {
  AddRequirements({climberSub});
  m_climberSubPtr = climberSub;
  m_isPartial = isPartial;
}

// Called when the command is initially scheduled.
void ClimberArmRaiseCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ClimberArmRaiseCmd::Execute() {
  m_climberSubPtr->setClimberArmPower(kClimberArmPower);
}

// Called once the command ends or is interrupted.
void ClimberArmRaiseCmd::End(bool interrupted) {
  m_climberSubPtr->setClimberArmPower(0.);
}

// Returns true when the command should end.
bool ClimberArmRaiseCmd::IsFinished() {
   int climberArmMaxHeight = kClimberArmMaxHeight;
  if(m_isPartial){
    climberArmMaxHeight /= 3; 
  }
  if (m_climberSubPtr->getClimberEncoder() > climberArmMaxHeight) {
    return true;
  }
  else {
    return false;
  }
}
