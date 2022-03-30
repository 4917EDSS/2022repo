// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberArmsLatchReleaseCmd.h"
#include <frc/RobotController.h>

ClimberArmsLatchReleaseCmd::ClimberArmsLatchReleaseCmd(ClimberSub *climberSub, bool armDirection) { //true = latch, false = release
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climberSub});
  m_climberSubPtr = climberSub;
  m_armDirection = armDirection;
}

// Called when the command is initially scheduled.
void ClimberArmsLatchReleaseCmd::Initialize() {
  m_startTime = frc::RobotController::GetFPGATime();
  if(m_armDirection) {
    m_climberSubPtr->foldArms();
  } else {
    m_climberSubPtr->unfoldArms();
  }
}

// Called repeatedly when this Command is scheduled to run
void ClimberArmsLatchReleaseCmd::Execute() {}

// Called once the command ends or is interrupted.
void ClimberArmsLatchReleaseCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimberArmsLatchReleaseCmd::IsFinished() {
  if((frc::RobotController::GetFPGATime() - m_startTime) > 1500000) {
    return true;
  }
  return false;
}
