// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MagazineJoystickCmd.h"

constexpr double kDeadband = 0.1;

MagazineJoystickCmd::MagazineJoystickCmd(IntakeSub* intakeSub, frc::Joystick *joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intakeSub});
  m_intakeSubPtr = intakeSub;
  m_joystickPtr = joystick;
}

double MagazineJoystickCmd::applyDeadband(double power) { 
  return (fabs(power) <= kDeadband) ? 0. : power; 
}

// Called when the command is initially scheduled.
void MagazineJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MagazineJoystickCmd::Execute() {
  bool isReversed;
  if (applyDeadband(-m_joystickPtr->GetThrottle()) > 0.0) 
    isReversed = true;
  else 
    isReversed = false;
  m_intakeSubPtr->enableFrontRollerIntakeMotor(isReversed);
}

// Called once the command ends or is interrupted.
void MagazineJoystickCmd::End(bool interrupted) {
  m_intakeSubPtr->disableFrontRollerIntakeMotor();
}

// Returns true when the command should end.
bool MagazineJoystickCmd::IsFinished() {
  return false;
}
