// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include "Constants.h"

namespace ShooterConstants {
  //All values need to be updated and tested
  constexpr double kDefaultLowerBinSpeed = 8500.0;   
  constexpr double kDefaultUpperBinSpeed = 16000.0;  
  constexpr double kShootTolerance = 500.0;
  constexpr double kDefaultF = 0.041;
  constexpr double kDefaultP = 0.18;
  constexpr double kDefaultI = 0.00001;
  constexpr double kDefaultD = 0;//0.5;
}

class ShooterSub : public frc2::SubsystemBase {
 public:
  double m_lowerBinSpeed;
  double m_upperBinSpeed;
  double m_kF; 
  double m_kP;
  double m_kI;
  double m_kD;
  double m_kNewF; 
  double m_kNewP;
  double m_kNewI;
  double m_kNewD;

  ShooterSub();
  void init(); // Resets all of the subsystem's hardware 
  void Periodic() override;

  void setPower(double power);
  double getSpeed();
  void autoVelocity(double velocity);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_shootMotor1{CanIds::kShootMotor1};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_shootMotor2{CanIds::kShootMotor2};
};
