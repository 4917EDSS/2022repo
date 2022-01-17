// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

////////////////////////////////////////////////////////////////////////////////////
// Test that we can create all of our hardware objects.
// DO NOT leave this enabled.  Testing only.
#if 0 // Set to 1 to test, 0 for robot deployment
#include <ctre/Phoenix.h>
ctre::phoenix::motorcontrol::can::WPI_TalonSRX myCanTalonSrxMotorController(30);    // TalonSRX via CAN (ID 30)
ctre::phoenix::motorcontrol::can::WPI_VictorSPX myCanVictorSpxMotorController(29);  // VictorSPX via CAN (ID 29)
ctre::phoenix::motorcontrol::can::WPI_TalonFX myCanTalonFXMotorController(28);      // TalonFX via CAN (ID 28)

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
rev::CANSparkMax myCanSparkMaxMotorController(27, rev::CANSparkMaxLowLevel::MotorType::kBrushless); // SparkMax via CAN (ID 27) for brushless motor (e.g. Neo)

#include <rev/ColorSensorV3.h>
rev::ColorSensorV3 myColorSensor(frc::I2C::kOnboard); // Colour sensor connected via Rio's I2C port

#include <AHRS.h>
AHRS myNavX2(frc::SPI::kMXP); // NavX/NavX2 Attitude and Heading Reference System (i.e. gyroscope) connected via Rio's MXP port
#endif
//
////////////////////////////////////////////////////////////////////////////////////

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
