// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetClimberArmCmd.h"
#include <frc/RobotController.h>

SetClimberArmCmd::SetClimberArmCmd(ClimberSub * armStatus, bool armDirection) { //true = latch, false = release
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({armStatus});
  m_armStatusPtr = armStatus;
  m_armDirection = armDirection;
}

// Called when the command is initially scheduled.
void SetClimberArmCmd::Initialize() {
  m_startTime = frc::RobotController::GetFPGATime();
  if (m_armDirection)
    m_armStatusPtr->foldArms();
  else
    m_armStatusPtr->unfoldArms();
}

// Called repeatedly when this Command is scheduled to run
void SetClimberArmCmd::Execute() {}

// Called once the command ends or is interrupted.
void SetClimberArmCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool SetClimberArmCmd::IsFinished() {
  if((frc::RobotController::GetFPGATime() - m_startTime) > 1500000) {
    return true;
  }
}
