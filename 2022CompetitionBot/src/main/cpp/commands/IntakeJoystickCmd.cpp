// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeJoystickCmd.h"

constexpr double kDeadband = 0.03;

IntakeJoystickCmd::IntakeJoystickCmd(IntakeSub* intakeSub, frc::Joystick *joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intakeSub});
  m_intakeSubPtr = intakeSub;
  m_joystickPtr = joystick;
}

double applyIntakeDeadband(double power) { 
  return (fabs(power) <= kDeadband) ? 0. : power; 
}

// Called when the command is initially scheduled.
void IntakeJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeJoystickCmd::Execute() {
  m_intakeSubPtr->setFrontRollerIntakeMotor(applyIntakeDeadband(-m_joystickPtr->GetY())); //Set power of front roller
  m_intakeSubPtr->setMagazineMotor(applyIntakeDeadband(-m_joystickPtr->GetThrottle())); //Also set power of magazine 
}

// Called once the command ends or is interrupted.
void IntakeJoystickCmd::End(bool interrupted) {
  m_intakeSubPtr->disableFrontRollerIntakeMotor();
}

// Returns true when the command should end.
bool IntakeJoystickCmd::IsFinished() {
  return false;
}
