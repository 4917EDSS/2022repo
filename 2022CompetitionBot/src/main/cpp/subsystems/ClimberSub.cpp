// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSub.h"

ClimberSub::ClimberSub() = default;

void ClimberSub::init() { //Reset all hardware to a safe state
}

// This method will be called once per scheduler run
void ClimberSub::Periodic() {}

void ClimberSub::setStationaryArmClimbPower(double power) {
    m_stationaryArmClimbMotor.Set(power);
}

void ClimberSub::setPivotingArmClimbPower(double power) {
    m_pivotingArmClimbMotor.Set(power);
}

void ClimberSub::setPivotingArmPivotPower(double power) {
    m_pivotingArmPivotMotor.Set(power);
}