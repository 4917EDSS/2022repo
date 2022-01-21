// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSub.h"

//Constants
constexpr int kFrontRollerIntakeMotorPower = 0.2;
constexpr int kMagazineMotorPower = 0.2;

IntakeSub::IntakeSub() = default;

// This method will be called once per scheduler run
void IntakeSub::Periodic() {}

//Enables the front roller intake motor.
void IntakeSub::enableFrontRollerIntakeMotor() {
  m_frontRollerIntakeMotor.Set(kFrontRollerIntakeMotorPower);
}

//Disables the front roller intake motor.
void IntakeSub::disableFrontRollerIntakeMotor() {
  m_frontRollerIntakeMotor.Set(0.0);
}

//Enables the magazine motor.
void IntakeSub::enableMagazineMotor() {
  m_magazineMotor.Set(kMagazineMotorPower);
}

//Disables the magazine motor.
void IntakeSub::disableMagazineMotor() {
  m_magazineMotor.Set(0.0);
}