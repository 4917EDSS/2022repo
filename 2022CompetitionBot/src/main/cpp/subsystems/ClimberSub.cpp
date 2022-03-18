// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSub.h"

ClimberSub::ClimberSub() {
    init();
}

void ClimberSub::init() { //Reset all hardware to a safe state
    // m_climbArmMotor.SetInverted(false);

    // m_climbArmMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
    // m_climbArmMotor.ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_5Ms); //Make sure to use SensorVelocityMeasPeriod because VelocityMeasPeriod is depricated
    // m_climbArmMotor.ConfigVelocityMeasurementWindow(4);
    // m_climbArmMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    setClimberArmPower(0.);
    zeroClimberEncoders();
}

// This method will be called once per scheduler run
void ClimberSub::Periodic() {}

void ClimberSub::setClimberArmPower(double power) {
    //m_climbArmMotor.Set(-power);
}

void ClimberSub::zeroClimberEncoders() {
    //m_climbArmMotor.SetSelectedSensorPosition(0);
}

double ClimberSub::getClimberEncoder() {
    //return -m_climbArmMotor.GetSelectedSensorPosition();
    return 0;
}

void ClimberSub::raiseArmSeparation() {
    m_armSeparationSolenoid.Set(true);
}

void ClimberSub::lowerArmSeparation() {
    m_armSeparationSolenoid.Set(false);
}

void ClimberSub::toggleArmSeparation() {
    m_armSeparationSolenoid.Set(!m_armSeparationSolenoid.Get());
}

