// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DrivetrainSub.h"

DrivetrainSub::DrivetrainSub() = default;

void DrivetrainSub::init() { //Reset all hardware to a safe state
    tankDrive(0.,0.);
    shiftDown();
    
    m_leftMotor1.SetInverted(false); //Find which motor is acutally reversed and you can remove the false SetInverted functions
    m_leftMotor2.SetInverted(true);
    m_leftMotor3.SetInverted(false);

    m_rightMotor1.SetInverted(false);
    m_rightMotor2.SetInverted(true);
    m_rightMotor3.SetInverted(false);
    //Plus anymore hardware added
}

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

void DrivetrainSub::arcadeDrive(double drivePwr, double rotatePwr) {
    m_drive.ArcadeDrive(drivePwr,rotatePwr);
}