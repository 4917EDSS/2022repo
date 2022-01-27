// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/Joystick.h>

#include "commands/ExampleCommand.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DrivetrainSub.h"
#include "subsystems/IntakeSub.h"


constexpr int kDriverControllorPort = 0;
constexpr int kOperatorControllorPort = 1;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void initSubsystems();

 private:
  // The robot's subsystems and commands are defined here...
  ExampleSubsystem m_subsystem;
  DrivetrainSub m_drivetrainSub;
  IntakeSub m_intakeSub;
  ExampleCommand m_autonomousCommand;
  frc::Joystick m_driverController{kDriverControllorPort};
  frc::Joystick m_operatorController{kOperatorControllorPort};

  void ConfigureButtonBindings();
};
