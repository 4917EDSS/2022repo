// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DrivetrainSub.h"

DrivetrainSub::DrivetrainSub() = default;

// This method will be called once per scheduler run
void DrivetrainSub::Periodic() {}

//Drive functions
void  DrivetrainSub::tankDrive(double lPower, double rPower) {
    m_leftMotor1.Set(lPower);
    m_leftMotor2.Set(-lPower);
    m_leftMotor3.Set(lPower);

    m_rightMotor1.Set(-rPower);
    m_rightMotor2.Set(rPower);
    m_rightMotor3.Set(-rPower);
}
void  DrivetrainSub::shiftUp() { //Gear shift up
    m_shifter.Set(1);
}
void  DrivetrainSub::shiftDown() { // Gear shift down
    m_shifter.Set(0);
}