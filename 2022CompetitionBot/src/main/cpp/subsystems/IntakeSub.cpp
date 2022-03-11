// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSub.h"

//Constants
constexpr double kFrontRollerIntakeMotorPower = 1.0;
constexpr double kMagazineMotorPower = 0.5;


IntakeSub::IntakeSub() :
  m_magazineFrontSensor{DioIds::kMagazineFrontSensor},
  m_magazineTopSensor{DioIds::kMagazineTopSensor} {
  init(); 
}

void IntakeSub::init() { //Reset all hardware to a safe state
  m_frontRollerIntakeMotor.SetInverted(false);
  m_magazineMotor.SetInverted(true);

  disableFrontRollerIntakeMotor();
  disableMagazineMotor();
  zeroIntakeEncoders();
  raiseIntake();
}

// This method will be called once per scheduler run
void IntakeSub::Periodic() {}

//Enables the front roller intake motor.
void IntakeSub::enableFrontRollerIntakeMotor(bool isReversed) {
  if(isReversed) {
    setFrontRollerIntakeMotor(-kFrontRollerIntakeMotorPower);
  } else {
    setFrontRollerIntakeMotor(kFrontRollerIntakeMotorPower);
  }
}

//Disables the front roller intake motor.
void IntakeSub::disableFrontRollerIntakeMotor() {
  setFrontRollerIntakeMotor(0.0);
}

//Set the front roller 
void IntakeSub::setFrontRollerIntakeMotor(double power) {
  m_frontRollerIntakeMotor.Set(power);
}

//Enables the magazine motor.
void IntakeSub::enableMagazineMotor(bool isReversed) {
  if(isReversed) {
    setMagazineMotor(-kMagazineMotorPower);
  } else {
    setMagazineMotor(kMagazineMotorPower);
  }
}

//Disables the magazine motor.
void IntakeSub::disableMagazineMotor() {
  setMagazineMotor(0.0);
}

void IntakeSub::setMagazineMotor(double power) {
  m_magazineMotor.Set(power);
}

void IntakeSub::zeroIntakeEncoders() {
  m_frontRollerIntakeEncoder.SetPosition(0.);
  m_magazineEncoder.SetPosition(0.);
}

void IntakeSub::raiseIntake() {
  m_armSolenoid1.Set(false);
}

void IntakeSub::lowerIntake() {
  m_armSolenoid1.Set(true);
}

void IntakeSub::toggleIntakeArm() {
  m_armSolenoid1.Set(!m_armSolenoid1.Get());
}

bool IntakeSub::isCargoAtMagazineBack(){
  if(m_magazineTopSensor.Get()) {
    return true;
  } else {
    return false;
  } 
}

bool IntakeSub::isCargoAtMagazineFront(){
  if(m_magazineFrontSensor.Get()) {
    return true;
  } else {
   return false;
  }
}


 