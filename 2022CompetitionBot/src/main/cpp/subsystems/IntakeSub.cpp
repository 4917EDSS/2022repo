// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSub.h"

//Constants
constexpr int kFrontRollerIntakeMotorPower = 0.2;
constexpr int kMagazineMotorPower = 0.2;


IntakeSub::IntakeSub() :
  m_frontIntakeSensor{DioIds::kFrontInTakeSensor},
  m_topMagazineSensor{DioIds::kMagazineTopSensor} {
  init(); 
}

void IntakeSub::init() { //Reset all hardware to a safe state
  disableFrontRollerIntakeMotor();
  disableMagazineMotor();
  zeroIntakeEncoders();
  raiseIntake();
  

  m_frontRollerIntakeMotor.SetInverted(false);
  m_magazineMotor.SetInverted(true);

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
  m_frontRollerIntakeEncoder.SetPosition(0.);
  m_magazineEncoder.SetPosition(0.);
}

void IntakeSub::raiseIntake() {
  m_armSolenoid1.Set(true);
  m_armSolenoid2.Set(true);
}

void IntakeSub::lowerIntake() {
  m_armSolenoid1.Set(false);
  m_armSolenoid2.Set(false);
}

bool IntakeSub::isCargoAtMagazineBack(){
  if( m_topMagazineSensor.Get()) {
    return true;
  } else {
    return false;
  } 
}

bool IntakeSub::isCargoAtMagazineFront(){
  if(m_frontIntakeSensor.Get()) {
    return true;
  } else {
   return false;
  }
}

void IntakeSub::toggleIntakeArm() {
  m_armSolenoid1.Set(!m_armSolenoid1.Get());
  m_armSolenoid2.Set(!m_armSolenoid2.Get());
}

 