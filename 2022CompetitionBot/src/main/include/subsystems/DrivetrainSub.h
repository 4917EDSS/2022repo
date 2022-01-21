// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc2/command/SubsystemBase.h> 

#include "Constants.h"

class DrivetrainSub : public frc2::SubsystemBase {
 public:
  DrivetrainSub();

  void tankDrive(double lPower, double rPower);
  void shiftUp();
  void shiftDown();
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_leftMotor1{CanIds::kLeftMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor2{CanIds::kLeftMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor3{CanIds::kLeftMotor3, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; 

  rev::CANSparkMax m_rightMotor1{CanIds::kRightMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor2{CanIds::kRightMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor3{CanIds::kRightMotor3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  
  frc::Solenoid m_shifter{frc::PneumaticsModuleType::CTREPCM, PneumaticIds::kShifter};
};
