// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/KillEverythingCmd.h"

KillEverythingCmd::KillEverythingCmd(ClimberSub* climberSub, DrivetrainSub* drivetrainSub, IntakeSub* intakeSub, ShooterSub* shooterSub) {
   AddRequirements({climberSub, drivetrainSub, intakeSub, shooterSub});
   m_drivetrainSubPtr = drivetrainSub;
}

// Called when the command is initially scheduled.
void KillEverythingCmd::Initialize() {
  m_drivetrainSubPtr->setBrakeMode(false);
}

// Called repeatedly when this Command is scheduled to run
void KillEverythingCmd::Execute() {}

// Called once the command ends or is interrupted.
void KillEverythingCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool KillEverythingCmd::IsFinished() {
  return true;
}
