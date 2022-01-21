// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class IntakeSub : public frc2::SubsystemBase {
 public:
  IntakeSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_frontRollerIntakeMotor{CanIds::kFrontRollerIntakeMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_magazineMotor{CanIds::kMagazineMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};


//#include <rev/CANSparkMaxLowLevel.h>
//rev::CANSparkMax myCanSparkMaxMotorController(27, rev::CANSparkMaxLowLevel::MotorType::kBrushless); // SparkMax via CAN (ID 27) for brushless motor (e.g. Neo)


};
