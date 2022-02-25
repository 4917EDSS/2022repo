// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSub.h"

ClimberSub::ClimberSub() {
    init();
}

void ClimberSub::init() { //Reset all hardware to a safe state
    m_stationaryArmClimbMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
    m_stationaryArmClimbMotor.ConfigVelocityMeasurementPeriod(ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_5Ms);
    m_stationaryArmClimbMotor.ConfigVelocityMeasurementWindow(4);

    setStationaryArmClimbPower(0.);
    setPivotingArmClimbPower(0.);
    setPivotingArmPivotPower(0.);
    zeroClimberEncoders();

    m_stationaryArmClimbMotor.SetInverted(false);
 //   m_pivotingArmClimbMotor.SetInverted(false);
 //   m_pivotingArmPivotMotor.SetInverted(false);

    homeArm();
    //Plus anymore hardware added 
}

// This method will be called once per scheduler run
void ClimberSub::Periodic() {}

void ClimberSub::setStationaryArmClimbPower(double power) {
    m_stationaryArmClimbMotor.Set(power);
}

void ClimberSub::setPivotingArmClimbPower(double power) {
//    m_pivotingArmClimbMotor.Set(power);
}

void ClimberSub::setPivotingArmPivotPower(double power) {
//    m_pivotingArmPivotMotor.Set(power);
}

void ClimberSub::zeroClimberEncoders() {
   // m_stationaryArmClimbMotor.GetEncoder().SetPosition(0.);
 //   m_pivotingArmClimbMotor.GetEncoder().SetPosition(0.);
 //   m_pivotingArmPivotMotor.GetEncoder().SetPosition(0.);
}

//Put arms in home position so that everything is in known state for beginning of game
void ClimberSub::homeArm(){
    //TBD 
}