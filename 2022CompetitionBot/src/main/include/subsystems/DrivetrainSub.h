// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h> 
#include <AHRS.h>

#include "Constants.h"



class DrivetrainSub : public frc2::SubsystemBase {
 public:
  DrivetrainSub();
  void init(); // Resets all of the subsystem's hardware 
  void Periodic() override;

  void tankDrive(double lPower, double rPower);
  void arcadeDrive(double drivePwr, double rotatePwr);

  void shiftUp();
  void shiftDown();
  void autoShift();
  bool isShiftedInHighGear();

  void zeroDrivetrainEncoders();
  void zeroHeading();

  double getLeftEncoderRaw();
  double getRightEncoderRaw();

  double getLeftEncoderDistanceM(); //meters
  double getRightEncoderDistanceM(); 

  double getLeftVelocity();
  double getRightVelocity();

  double getHeading();
  double getTurnRate();
  void setIsAutoShift(bool autoShiftActive);
  bool m_isAutoShift;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_leftMotor1{CanIds::kLeftMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor2{CanIds::kLeftMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor3{CanIds::kLeftMotor3, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; 

  rev::CANSparkMax m_rightMotor1{CanIds::kRightMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor2{CanIds::kRightMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor3{CanIds::kRightMotor3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  
  rev::SparkMaxRelativeEncoder m_leftMotorEncoder{m_leftMotor1.GetEncoder()};
  rev::SparkMaxRelativeEncoder m_rightMotorEncoder{m_rightMotor1.GetEncoder()};

  frc::MotorControllerGroup m_leftMotors{m_leftMotor1, m_leftMotor2, m_leftMotor3};
  frc::MotorControllerGroup m_rightMotors{m_rightMotor1, m_rightMotor2, m_rightMotor3};
  frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};
  frc::Solenoid m_shifter{frc::PneumaticsModuleType::CTREPCM, PneumaticIds::kShifter};

  AHRS m_gyro{frc::SPI::kMXP};

  
  
  double getEncoderRotationsToMeterFactor();
};
