// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberArmsLatchReleaseCmd.h"
#include <frc/RobotController.h>

ClimberArmsLatchReleaseCmd::ClimberArmsLatchReleaseCmd(ClimberSub *climberSub, bool isFoldIn, bool isDelayAtStart, bool isDelayAtEnd) { 
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climberSub});
  m_climberSubPtr = climberSub;
  m_isFoldIn = isFoldIn;
  m_isDelayAtStart = isDelayAtStart;
  m_isDelayAtEnd = isDelayAtEnd;
}

// Called when the command is initially scheduled.
void ClimberArmsLatchReleaseCmd::Initialize() {
  m_startTime = frc::RobotController::GetFPGATime();
  m_hasExecuted = false;
  m_executeCmd = false;
}

// Called repeatedly when this Command is scheduled to run
void ClimberArmsLatchReleaseCmd::Execute() {
  if(m_isDelayAtStart){
    if(!m_hasExecuted && (frc::RobotController::GetFPGATime() - m_startTime) > 1500000){
      m_executeCmd = true;
    }
  }
  else if (!m_hasExecuted) {
    m_executeCmd = true;
  }
  if(m_executeCmd){
    m_startTime = frc::RobotController::GetFPGATime();//resetting start time
    if(m_isFoldIn) {
      m_climberSubPtr->foldArms();
    } else {
      m_climberSubPtr->unfoldArms();
    }
    m_hasExecuted = true;
    m_executeCmd = false;
  }
}

// Called once the command ends or is interrupted.
void ClimberArmsLatchReleaseCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimberArmsLatchReleaseCmd::IsFinished() {
  int delayTime = 1500000;
  if (m_isDelayAtEnd){
    delayTime *= 2; //Delay it by 2 times as much
  }
  if((frc::RobotController::GetFPGATime() - m_startTime) > delayTime) {
    return true;
  }
  return false;
}
