// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include "Constants.h"

class ClimberSub : public frc2::SubsystemBase {
 public:
  ClimberSub();
  void init(); // Resets all of the subsystem's hardware 
  void Periodic() override;
  
  void unfoldArms();
  bool getArmStatus();
  void foldArms();
  void toggleArmSeparation();

  void setClimberArmPower(double power);
  void zeroClimberEncoders(); 
  double getClimberEncoder();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_climbArmMotor{CanIds::kClimberArmbMotor};

  frc::Solenoid m_armSeparationSolenoid{frc::PneumaticsModuleType::CTREPCM, PneumaticIds::kArmSeparation};
};

