// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/Joystick.h>

#include "subsystems/DrivetrainSub.h"
#include "subsystems/IntakeSub.h"
#include "subsystems/ClimberSub.h"
#include "subsystems/ShooterSub.h"
#include "commands/AutoDoNothingCmd.h"

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
  void updateDashboard();

 private:
  // The robot's subsystems and commands are defined here...
  DrivetrainSub m_drivetrainSub;
  IntakeSub m_intakeSub;
  ShooterSub m_shooterSub;
  ClimberSub m_climberSub;
  AutoDoNothingCmd m_autonomousCommand;
  frc::Joystick m_driverController{kDriverControllorPort};
  frc::Joystick m_operatorController{kOperatorControllorPort};

  void ConfigureButtonBindings();
};
