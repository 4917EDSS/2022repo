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
}

// Called when the command is initially scheduled.
void ShootCargoCmd::Initialize() {
  m_shooterSubPtr->setPower(0);
  m_previousI = 0;
}

// Called repeatedly when this Command is scheduled to run
void ShootCargoCmd::Execute() {
  double targetSpeed;
  if(m_isUpperGoal) {
    targetSpeed = m_shooterSubPtr->m_upperBinSpeed;
  } else {
    targetSpeed = m_shooterSubPtr->m_lowerBinSpeed;
  }
  double speed = m_shooterSubPtr->getSpeed();
  double i = m_previousI;
  double difference = (targetSpeed - speed) / targetSpeed;
 
  i += difference;
  m_shooterSubPtr->setPower((difference * m_shooterSubPtr->m_kP) + (i * m_shooterSubPtr->m_kI));
  m_previousI = i;

  std::cout << "Shoot " << targetSpeed << " " << speed << " " << fabs(targetSpeed - speed) << "\n";
  if(fabs(targetSpeed - speed) < 5000) {
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
