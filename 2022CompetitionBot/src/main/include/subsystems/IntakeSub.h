// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>

#include "Constants.h"

class IntakeSub : public frc2::SubsystemBase {
 public:
  IntakeSub();

  void init(); // Resets all of the subsystem's hardware 
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void enableFrontRollerIntakeMotor(bool isReversed);
  void setFrontRollerIntakeMotor(double power); //Set power of intake motor
  void disableFrontRollerIntakeMotor();
  void enableMagazineMotor(bool isReversed);
  void setMagazineMotor(double power); //Set power of magazine motor
  void disableMagazineMotor();
  void zeroIntakeEncoders();
  void raiseIntake();
  void lowerIntake();
  bool isCargoAtMagazineFront();
  bool isCargoAtMagazineBack();
  void toggleIntakeArm();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_frontRollerIntakeMotor{CanIds::kFrontRollerIntakeMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_magazineMotor{CanIds::kMagazineMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  frc::Solenoid m_armSolenoid1{frc::PneumaticsModuleType::CTREPCM, PneumaticIds::kArm1};
  frc::Solenoid m_armSolenoid2{frc::PneumaticsModuleType::CTREPCM, PneumaticIds::kArm2};

  frc::DigitalInput m_frontIntakeSensor;
  frc::DigitalInput m_topMagazineSensor;

  rev::SparkMaxRelativeEncoder m_frontRollerIntakeEncoder{m_frontRollerIntakeMotor.GetEncoder()};
  rev::SparkMaxRelativeEncoder m_magazineEncoder{m_magazineMotor.GetEncoder()};
};
