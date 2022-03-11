// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SpinFlywheelCmd.h"

SpinFlywheelCmd::SpinFlywheelCmd(ShooterSub* shooterSub, bool isUpperGoal) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
  m_shooterSubPtr = shooterSub;
  m_isUpperGoal = isUpperGoal;
}

// Called when the command is initially scheduled.
void SpinFlywheelCmd::Initialize() {
  m_shooterSubPtr->setPower(0);
}

// Called repeatedly when this Command is scheduled to run
void SpinFlywheelCmd::Execute() {
  int targetSpeed;

  // Get the latest target speed (can be changed on Dashboard)
  if(m_isUpperGoal) {
    targetSpeed = m_shooterSubPtr->m_upperBinSpeed;
  } else {
    targetSpeed = m_shooterSubPtr->m_lowerBinSpeed;
  }


  double currentSpeed = m_shooterSubPtr->getSpeed();
  double difference = (targetSpeed*2.0 - currentSpeed) / targetSpeed;
  m_shooterSubPtr->setPower(difference * m_shooterSubPtr->m_kP);
}

// Called once the command ends or is interrupted.
void SpinFlywheelCmd::End(bool interrupted) {
  m_shooterSubPtr->setPower(0);
}

// Returns true when the command should end.
bool SpinFlywheelCmd::IsFinished() {
  return false;
}
