// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <constants.h>

class ExampleSubsystem : public frc2::SubsystemBase {
 public:
  ExampleSubsystem();
  void tankDrive(double leftPower, double rightPower);
    


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
rev::CANSparkMax m_leftMotor1{CanIds::kLeftMotor1, 
                          rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor1{CanIds::kRightMotor1, 
                          rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
