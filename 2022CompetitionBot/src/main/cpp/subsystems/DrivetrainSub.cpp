// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DrivetrainSub.h"

constexpr double kShiftUpSpeed = 2.35; //meters per second
constexpr double kShiftDownSpeed = 1.15; 

constexpr double kEncoderRotationsToMetersLowGear = 204.44/5.0; //Find these actual values
constexpr double kEncoderRotationsToMetersHighGear = 129.5/5.0;
constexpr bool kGyroReversed = true;

DrivetrainSub::DrivetrainSub() {
  init(); 
}

void DrivetrainSub::init() { //Reset all hardware to a safe state
    m_leftMotor1.SetInverted(true); //Find which motor is acutally reversed and you can remove the false SetInverted functions
    m_leftMotor2.SetInverted(true);
    m_leftMotor3.SetInverted(true);

    m_rightMotor1.SetInverted(false);
    m_rightMotor2.SetInverted(false);
    m_rightMotor3.SetInverted(false);

    zeroDrivetrainEncoders();
    tankDrive(0., 0.);
    shiftDown();

    m_gyro.Reset();
}

void DrivetrainSub::Periodic() {}

//Drive functions
void  DrivetrainSub::tankDrive(double lPower, double rPower) {
    m_drive.TankDrive(lPower, rPower);
}

void DrivetrainSub::arcadeDrive(double drivePwr, double rotatePwr) {
    m_drive.ArcadeDrive(drivePwr, rotatePwr);
}

void  DrivetrainSub::shiftUp() { //Gear shift up
    m_shifter.Set(1);
}

void  DrivetrainSub::shiftDown() { // Gear shift down
    m_shifter.Set(0);
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

bool DrivetrainSub::isShiftedInHighGear() {
    return m_shifter.Get();
}

void DrivetrainSub::zeroDrivetrainEncoders() {
    m_leftMotorEncoder.SetPosition(0.);
    m_rightMotorEncoder.SetPosition(0.);
}

double DrivetrainSub::getLeftEncoderRaw() { //Returns rotations (1 full motor rotation = 1.)
    return -m_leftMotorEncoder.GetPosition(); //verify which encoder is reversed
}

double DrivetrainSub::getRightEncoderRaw() { 
    return m_rightMotorEncoder.GetPosition();
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
    return m_leftMotorEncoder.GetVelocity() * getEncoderRotationsToMeterFactor() * 60.; //In meters per second
}

double DrivetrainSub::getRightVelocity() {
    return m_rightMotorEncoder.GetVelocity() * getEncoderRotationsToMeterFactor() * 60.; //In meters per second
}

double DrivetrainSub::getHeading() {
    return std::remainder(m_gyro.GetAngle(),360.) * (kGyroReversed ? -1. : 1.);
}

double DrivetrainSub::getTurnRate () {
    return m_gyro.GetRate() * (kGyroReversed ? -1. : 1.);
}