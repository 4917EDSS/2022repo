// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootCargoCmd.h"
#include "iostream"

ShootCargoCmd::ShootCargoCmd(ShooterSub* shooterSub, IntakeSub* intakeSub, bool isUpperGoal) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
  AddRequirements({intakeSub});

  m_shooterSubPtr = shooterSub;
  m_intakeSubPtr = intakeSub;
  m_isUpperGoal = isUpperGoal;

  if(m_isUpperGoal) {
    m_targetSpeed = m_shooterSubPtr->m_upperBinSpeed;
  } else {
    m_targetSpeed = m_shooterSubPtr->m_lowerBinSpeed;
  }
}

// Called when the command is initially scheduled.
void ShootCargoCmd::Initialize() {
  m_shooterSubPtr->autoVelocity(m_targetSpeed);
}

// Called repeatedly when this Command is scheduled to run
void ShootCargoCmd::Execute() {
  if(fabs(m_targetSpeed - m_shooterSubPtr->getSpeed()) < ShooterConstants::kShootTolerance) {
    m_intakeSubPtr->enableMagazineMotor(false);
  } else {
    m_intakeSubPtr->disableMagazineMotor();
  }
}

// Called once the command ends or is interrupted.
void ShootCargoCmd::End(bool interrupted) {
  m_shooterSubPtr->setPower(0);
  m_intakeSubPtr->disableMagazineMotor();
}

// Returns true when the command should end.
bool ShootCargoCmd::IsFinished() {
  return false;
}
