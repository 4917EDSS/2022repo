// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

namespace ShooterConstants {
  //All values need to be updated and tested
  constexpr double kMaxRPM = 21750;
  constexpr double kLowerBinSpeed = 15000;
  constexpr double kUpperBinSpeed = 17800;   
  constexpr double kMaxSpeed = 20500; 
  static_assert(kUpperBinSpeed < kMaxSpeed, "Upper Bin must be less than max speed");
  static_assert(kLowerBinSpeed < kMaxSpeed, "Lower Bin must be less than max speed");
}

class ShooterSub : public frc2::SubsystemBase {
 public:
  ShooterSub();

  void init(); // Resets all of the subsystem's hardware 
  void setPower(double power);
  void zeroShooterEncoders();
  double getSpeed();
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax m_shootMotor1{CanIds::kShootMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_shootMotor2{CanIds::kShootMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
};
