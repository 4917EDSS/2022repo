// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootCargoCmd.h"

constexpr double kTargetSpeed = 7000;
constexpr double kPowerAdded = 0.04;
constexpr double kP = 0.1;
constexpr double kI = 0.1;

ShootCargoCmd::ShootCargoCmd(ShooterSub* shooterSub, IntakeSub* intakeSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
  m_shooterSubPtr = shooterSub;

  AddRequirements({intakeSub});
  m_intakeSubPtr = intakeSub;
}

// Called when the command is initially scheduled.
void ShootCargoCmd::Initialize() {
  m_shooterSubPtr->setPower(0);
  m_previousI = 0;
}

// Called repeatedly when this Command is scheduled to run
void ShootCargoCmd::Execute() {
  double speed = m_shooterSubPtr->getSpeed();
  double i = m_previousI;
  double difference = (kTargetSpeed - speed) / kTargetSpeed;
 
  i += difference;
  m_shooterSubPtr->setPower((difference * kP) + (i * kI));
  m_previousI = i;

  if(fabs(kTargetSpeed - speed) < 100) {
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
