// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SpinFlywheelCmd.h"

SpinFlywheelCmd::SpinFlywheelCmd(ShooterSub* shooterSub, bool isUpper) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_shooterSub = shooterSub;
  m_isUpper = isUpper;
  m_targetSpeed = 0.;
}

// Called when the command is initially scheduled.
void SpinFlywheelCmd::Initialize() {
  m_targetSpeed =(m_isUpper ? ShooterConstants::kUpperBinSpeed : ShooterConstants::kLowerBinSpeed);
  double feed = m_targetSpeed / ShooterConstants::kMaxRPM;
  m_shooterSub->setPower(0.1); ////////////////////////////////////////////////// temporarily setting to 10%
}

// Called repeatedly when this Command is scheduled to run
void SpinFlywheelCmd::Execute() {}

// Called once the command ends or is interrupted.
void SpinFlywheelCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool SpinFlywheelCmd::IsFinished() {
  if(m_shooterSub->getSpeed() >= (m_targetSpeed * 0.9)) {
    return true;
  }
  else {
    return false;
  }
}
