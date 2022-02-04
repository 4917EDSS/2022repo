// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DrivetrainSub.h"

constexpr double kShiftUpSpeed = 2.35; //meters per second
constexpr double kShiftDownSpeed = 1.15; 

constexpr double kEncoderRotationsToMetersLowGear = 1.; //Find these actual values
constexpr double kEncoderRotationsToMetersHighGear = 1.;

DrivetrainSub::DrivetrainSub() {
  init(); 
}

void DrivetrainSub::init() { //Reset all hardware to a safe state
    tankDrive(0., 0.);
    shiftDown();
    zeroDrivetrainEncoders();

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
void DrivetrainSub::arcadeDrive(double drivePwr, double rotatePwr) {
    m_drive.ArcadeDrive(drivePwr,rotatePwr);
}
void  DrivetrainSub::shiftUp() { //Gear shift up
    m_shifter.Set(1);
}
void  DrivetrainSub::shiftDown() { // Gear shift down
    m_shifter.Set(0);
}

bool DrivetrainSub::isShiftedInHighGear() {
    return m_shifter.Get();
}
void DrivetrainSub::autoShift() {
    //get speed from robot
    double averageWheelSpeed = (getLeftVelocity() + getRightVelocity()) / 2.;

    if(fabs(averageWheelSpeed) > kShiftUpSpeed) {
        shiftUp();
    }
    else if(fabs(averageWheelSpeed) < kShiftDownSpeed) {
        shiftDown();
    }
}

void DrivetrainSub::zeroDrivetrainEncoders() {
    m_leftMotor1.GetEncoder().SetPosition(0.);
    m_leftMotor2.GetEncoder().SetPosition(0.);
    m_leftMotor3.GetEncoder().SetPosition(0.);

    m_rightMotor1.GetEncoder().SetPosition(0.);
    m_rightMotor2.GetEncoder().SetPosition(0.);
    m_rightMotor3.GetEncoder().SetPosition(0.);
}

double DrivetrainSub::getLeftEncoderRaw() { //Returns rotations (1 full motor rotation = 1.)
    return -m_leftMotor1.GetEncoder().GetPosition(); //verify which encoder is reversed
}
double DrivetrainSub::getRightEncoderRaw() { 
    return m_rightMotor1.GetEncoder().GetPosition();
}

double DrivetrainSub::getLeftEncoderDistanceM() {
    return getLeftEncoderRaw() * getEncoderRotationsToMeterFactor();
}
double DrivetrainSub::getRightEncoderDistanceM() {
    return getLeftEncoderRaw() * getEncoderRotationsToMeterFactor();
}

double DrivetrainSub::getEncoderRotationsToMeterFactor() {
    return (isShiftedInHighGear()) ? kEncoderRotationsToMetersHighGear : kEncoderRotationsToMetersLowGear;
}

double DrivetrainSub::getLeftVelocity() {
    return m_leftMotor1.GetEncoder().GetVelocity() * getEncoderRotationsToMeterFactor() * 60.; //In meters per second
}
double DrivetrainSub::getRightVelocity() {
    return m_rightMotor1.GetEncoder().GetVelocity() * getEncoderRotationsToMeterFactor() * 60.; //In meters per second
}
