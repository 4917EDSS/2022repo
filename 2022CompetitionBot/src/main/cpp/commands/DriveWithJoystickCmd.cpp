// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveWithJoystickCmd.h"


DriveWithJoystickCmd::DriveWithJoystickCmd(DrivetrainSub *drivetrainSub, frc::Joystick *joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrainSubPtr = drivetrainSub;
  m_joystickPtr = joystick;
}

// Called when the command is initially scheduled.
void DriveWithJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
  m_drivetrainSubPtr->tankDrive(m_joystickPtr->GetY(), m_joystickPtr->GetThrottle());
}

// Called once the command ends or is interrupted.
void DriveWithJoystickCmd::End(bool interrupted) {
  //This is a default command that doesn't end
}

// Returns true when the command should end.
bool DriveWithJoystickCmd::IsFinished() {
  return false;
}
