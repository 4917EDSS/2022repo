// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveWithJoystickCmd.h"

constexpr int kSensitivityPower = 2;
constexpr double kDeadband = 0.03;
constexpr double kMaxForwardAccl = 0.045;
constexpr double kMaxTurnAccl = 0.08;

DriveWithJoystickCmd::DriveWithJoystickCmd(DrivetrainSub *drivetrainSub, frc::Joystick *joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
  m_drivetrainSubPtr = drivetrainSub;
  m_joystickPtr = joystick;
}

double adjustSensitivity(double power) {
  double dir = (power < 0) ? -1. : 1.; 
  power = pow(power,kSensitivityPower) * dir;

  return power;
}

double applyDeadband(double power) { 
  return (fabs(power) <= kDeadband) ? 0. : power; 
}

double capAcceleration(double power,double prevPower,double cap) {
  double dir = (prevPower-power)/fabs(prevPower-power); 
  if(fabs(power-prevPower) > cap) {
    power = prevPower+(cap*dir); 
  }
  return power;
}
// Called when the command is initially scheduled.
void DriveWithJoystickCmd::Initialize() {
  m_prevFwdPower = 0.;
  m_prevTurnPower = 0.;
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
  double fwdPower = m_joystickPtr->GetY();
  double turnPower = m_joystickPtr->GetThrottle();

  fwdPower = adjustSensitivity(fwdPower);
  turnPower = adjustSensitivity(turnPower);

  fwdPower = capAcceleration(fwdPower, m_prevFwdPower,kMaxForwardAccl);
  turnPower = capAcceleration(turnPower, m_prevTurnPower,kMaxTurnAccl);

  fwdPower = applyDeadband(fwdPower);
  turnPower = applyDeadband(turnPower);

  m_drivetrainSubPtr->arcadeDrive(fwdPower, turnPower);
  m_prevFwdPower = fwdPower;
  m_prevTurnPower = turnPower;

  m_drivetrainSubPtr->autoShift();
}

// Called once the command ends or is interrupted.
void DriveWithJoystickCmd::End(bool interrupted) {
  //This is a default command that doesn't end

}

// Returns true when the command should end.
bool DriveWithJoystickCmd::IsFinished() {
  return false;
}
