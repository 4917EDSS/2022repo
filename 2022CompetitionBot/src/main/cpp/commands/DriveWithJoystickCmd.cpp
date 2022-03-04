// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveWithJoystickCmd.h"

constexpr int kSensitivityPower = 2;
constexpr double kDeadband = 0.03;
constexpr double kMaxForwardAccl = 0.1;
constexpr double kMaxTurnAccl = 0.08;

DriveWithJoystickCmd::DriveWithJoystickCmd(DrivetrainSub *drivetrainSub, frc::Joystick *joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_joystickPtr = joystick;
  m_curFwdPower = 0.;
  m_curTurnPower = 0.;

}

double adjustSensitivity(double power) {
  double dir = (power < 0) ? -1. : 1.; 
  power = pow(power,kSensitivityPower) * dir; 

  return power;
}

double applyDeadband(double power) { 
  return (fabs(power) <= kDeadband) ? 0. : power; 
}

double capAcceleration(double targetPower, double curPower, double maxAcceleration) {
  // double accelerationDirection = (curPower - targetPower) / fabs(curPower - targetPower);
  bool positiveAcceleration;
  double newPower = targetPower; // start by assuming that we don't need to step
  if(curPower - targetPower < 0) {
    positiveAcceleration = true;
  } else {
    positiveAcceleration = false;
  }

  // check if we do need to do a step 
  if(fabs(targetPower - curPower) > maxAcceleration) {
    if(positiveAcceleration) {
      newPower = curPower + maxAcceleration;
    } else {
      newPower = curPower - maxAcceleration;
    } 
  }

  return newPower;
}
// Called when the command is initially scheduled.
void DriveWithJoystickCmd::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
  double fwdPower = -m_joystickPtr->GetY();
  double turnPower = m_joystickPtr->GetZ();

  fwdPower = adjustSensitivity(fwdPower);
  turnPower = adjustSensitivity(turnPower);

  fwdPower = capAcceleration(fwdPower, m_curFwdPower, kMaxForwardAccl);
  turnPower = capAcceleration(turnPower, m_curTurnPower, kMaxTurnAccl);

  fwdPower = applyDeadband(fwdPower);
  turnPower = applyDeadband(turnPower);

  m_drivetrainSubPtr->arcadeDrive(fwdPower, turnPower);
  m_curFwdPower = fwdPower;
  m_curTurnPower = turnPower;

  // m_drivetrainSubPtr->autoShift();
}

// Called once the command ends or is interrupted.
void DriveWithJoystickCmd::End(bool interrupted) {
  //This is a default command that doesn't end

}

// Returns true when the command should end.
bool DriveWithJoystickCmd::IsFinished() {
  return false;
}
