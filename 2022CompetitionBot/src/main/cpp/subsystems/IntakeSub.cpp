// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSub.h"

//Constants
constexpr int kFrontRollerIntakeMotorPower = 0.2;
constexpr int kMagazineMotorPower = 0.2;


IntakeSub::IntakeSub() {
  init(); 
}

void IntakeSub::init() { //Reset all hardware to a safe state
  disableFrontRollerIntakeMotor();
  disableMagazineMotor();
  zeroIntakeEncoders();

  m_frontRollerIntakeMotor.SetInverted(false);
  m_magazineMotor.SetInverted(false);
  //Plus anymore hardware added 
}

// This method will be called once per scheduler run
void IntakeSub::Periodic() {}

//Enables the front roller intake motor.
void IntakeSub::enableFrontRollerIntakeMotor(bool isReversed) {
  if (isReversed) {
    m_frontRollerIntakeMotor.Set(-kFrontRollerIntakeMotorPower);
  } else {
    m_frontRollerIntakeMotor.Set(kFrontRollerIntakeMotorPower);
  }
}

//Disables the front roller intake motor.
void IntakeSub::disableFrontRollerIntakeMotor() {
  m_frontRollerIntakeMotor.Set(0.0);
}

//Enables the magazine motor.
void IntakeSub::enableMagazineMotor(bool isReversed) {
  if (isReversed) {
    m_magazineMotor.Set(-kMagazineMotorPower);
  } else {
    m_magazineMotor.Set(kMagazineMotorPower);
  }
}

//Disables the magazine motor.
void IntakeSub::disableMagazineMotor() {
  m_magazineMotor.Set(0.0);
}

void IntakeSub::zeroIntakeEncoders() {
  m_frontRollerIntakeMotor.GetEncoder().SetPosition(0.);
  m_magazineMotor.GetEncoder().SetPosition(0.);
}