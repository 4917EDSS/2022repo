// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateRobotCmd.h"

RotateRobotCmd::RotateRobotCmd(DrivetrainSub *DrivetrainSub, double angle) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RotateRobotCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RotateRobotCmd::Execute() {}

// Called once the command ends or is interrupted.
void RotateRobotCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool RotateRobotCmd::IsFinished() {
  return false;
}
