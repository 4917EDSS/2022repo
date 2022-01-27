// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class ClimberSub : public frc2::SubsystemBase {
 public:
  ClimberSub();
 
  void init(); // Resets all of the subsystem's hardware 
  void setStationaryArmClimbPower(double power);
  void setPivotingArmClimbPower(double power);
  void setPivotingArmPivotPower(double power);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax m_stationaryArmClimbMotor{CanIds::kStationaryArmClimbMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_pivotingArmClimbMotor{CanIds::kPivotingArmClimbMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_pivotingArmPivotMotor{CanIds::kPivotingArmPivotMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
};

