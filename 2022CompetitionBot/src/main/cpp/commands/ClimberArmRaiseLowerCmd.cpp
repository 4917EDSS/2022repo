// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberArmRaiseLowerCmd.h"

constexpr double kClimberArmPower = 0.75;
constexpr int kClimberArmMaxHeight = 195000;
constexpr int kClimberArmMinHeight = 0;

ClimberArmRaiseLowerCmd::ClimberArmRaiseLowerCmd(ClimberSub* climberSub, bool climberDirection) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climberSub});
  m_climberSubPtr = climberSub;
  m_climberDirection = climberDirection;
}

// Called when the command is initially scheduled.
void ClimberArmRaiseLowerCmd::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ClimberArmRaiseLowerCmd::Execute() {
  if (m_climberDirection && (m_climberSubPtr->getClimberEncoder() < kClimberArmMaxHeight)) {
    m_climberSubPtr->setClimberArmPower(kClimberArmPower);
  } else if (!m_climberDirection && (m_climberSubPtr->getClimberEncoder() > kClimberArmMinHeight)) {
    m_climberSubPtr->setClimberArmPower(-kClimberArmPower);
  } else {
    m_climberSubPtr->setClimberArmPower(0);
  }
}

// Called once the command ends or is interrupted.
void ClimberArmRaiseLowerCmd::End(bool interrupted) {
  m_climberSubPtr->setClimberArmPower(0.);
}

// Returns true when the command should end.
bool ClimberArmRaiseLowerCmd::IsFinished() {
  return false;
}
