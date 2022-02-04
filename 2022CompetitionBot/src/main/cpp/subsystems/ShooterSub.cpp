// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSub.h"

ShooterSub::ShooterSub() {
    init(); 
}

void ShooterSub::init() { //Reset all hardware to a safe state
    setPower(0.);
    zeroShooterEncoders();

    m_shootMotor1.SetInverted(false);
    m_shootMotor2.SetInverted(true); 
    //Plus anymore hardware added 
}

// This method will be called once per scheduler run
void ShooterSub::Periodic() {}

// Providing power to the shooter motors
void ShooterSub::setPower(double power) {
    m_shootMotor1.Set(power);
    m_shootMotor2.Set(power); 
}

void ShooterSub::zeroShooterEncoders() {
    m_shootMotor1.GetEncoder().SetPosition(0.);
    m_shootMotor2.GetEncoder().SetPosition(0.);
}