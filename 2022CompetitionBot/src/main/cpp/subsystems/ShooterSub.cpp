// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSub.h"

constexpr int kTimeoutMs = 30;

ShooterSub::ShooterSub() {
    init(); 
}

void ShooterSub::init() { //Reset all hardware to a safe state
    m_shootMotor1.SetInverted(false);
    m_shootMotor2.SetInverted(true); 
    setPower(0.);
    m_shootMotor1.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
    m_shootMotor1.ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_5Ms);
    m_shootMotor1.ConfigVelocityMeasurementWindow(4);
    m_shootMotor1.ConfigVoltageCompSaturation(12);
    m_shootMotor1.EnableVoltageCompensation(true);

    m_shootMotor2.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
    m_shootMotor2.ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_5Ms);
    m_shootMotor2.ConfigVelocityMeasurementWindow(4);
    m_shootMotor2.ConfigVoltageCompSaturation(12);
    m_shootMotor2.EnableVoltageCompensation(true);

    m_lowerBinSpeed = ShooterConstants::kDefaultLowerBinSpeed;
    m_upperBinSpeed = ShooterConstants::kDefaultUpperBinSpeed;
    m_kNewF = m_kF = ShooterConstants::kDefaultF;
    m_kNewP = m_kP = ShooterConstants::kDefaultP;
    m_kNewI = m_kI = ShooterConstants::kDefaultI;
    m_kNewD = m_kD = ShooterConstants::kDefaultD;

    // m_shootMotor2.Follow(m_shootMotor1);
    m_shootMotor1.Config_kF(0, m_kF, kTimeoutMs);
    m_shootMotor1.Config_kP(0, m_kP, kTimeoutMs);
    m_shootMotor1.Config_kI(0, m_kI, kTimeoutMs);
    m_shootMotor1.Config_kD(0, m_kD, kTimeoutMs);

    m_shootMotor2.Config_kF(0, m_kF, kTimeoutMs);
    m_shootMotor2.Config_kP(0, m_kP, kTimeoutMs);
    m_shootMotor2.Config_kI(0, m_kI, kTimeoutMs);
    m_shootMotor2.Config_kD(0, m_kD, kTimeoutMs);
}

// This method will be called once per scheduler run
void ShooterSub::Periodic() {
    // if (m_kNewF != m_kF) {
    //     m_kF = m_kNewF;
    //     m_shootMotor1.Config_kF(0, m_kF, kTimeoutMs);
    //     m_shootMotor2.Config_kF(0, m_kF, kTimeoutMs);
    // }

    // if (m_kNewP != m_kP) {
    //     m_kP = m_kNewP;
    //     m_shootMotor1.Config_kP(0, m_kP, kTimeoutMs);
    //     m_shootMotor2.Config_kP(0, m_kP, kTimeoutMs);
    // }

    // if (m_kNewI != m_kI) {
    //     m_kI = m_kNewI;
    //     m_shootMotor1.Config_kI(0, m_kI, kTimeoutMs);
    //     m_shootMotor2.Config_kI(0, m_kI, kTimeoutMs);
    // }
    
    // if (m_kNewD != m_kD) {
    //     m_kD = m_kNewD;
    //     m_shootMotor1.Config_kD(0, m_kD, kTimeoutMs);
    //     m_shootMotor2.Config_kD(0, m_kD, kTimeoutMs);
    // }
}

// Providing power to the shooter motors
void ShooterSub::setPower(double power) {
    m_shootMotor1.Set(power);
    m_shootMotor2.Set(power); 
}

void ShooterSub::autoVelocity(double velocity) {
    m_shootMotor1.Set(ControlMode::Velocity, velocity); 
    m_shootMotor2.Set(ControlMode::Velocity, velocity); 
}

double ShooterSub::getSpeed() {
    return m_shootMotor1.GetSelectedSensorVelocity();
}

double ShooterSub::getShotPower(double distanceM) {
    int mapSize = 4;
    double distanceMap[mapSize][2] = {{2.25, 14900}, {2.5, 15500}, {2.75, 16750}, {3, 18000}};

    if (distanceM <= distanceMap[0][0]) {
        return distanceMap[0][1];
    }

    for(int i = 1; i < mapSize; i++) {
        if (distanceM <= distanceMap[i][0]) {
            double ratio = 1 - ((distanceMap[i][0] - distanceM) / (distanceMap[i][0] - distanceMap[i-1][0]));
            return (distanceMap[i-1][1] + ratio * (distanceMap[i][1] - distanceMap[i-1][1]));
        }
    }
}
