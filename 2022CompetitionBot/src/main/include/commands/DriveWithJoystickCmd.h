// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include "subsystems/DrivetrainSub.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveWithJoystickCmd
    : public frc2::CommandHelper<frc2::CommandBase, DriveWithJoystickCmd> {
 public:
  DriveWithJoystickCmd(DrivetrainSub *drivetrainSub, frc::Joystick *joystick);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  DrivetrainSub *m_drivetrainSubPtr;
  frc::Joystick *m_joystickPtr;
  double m_curFwdPower;
  double m_curTurnPower;

  double adjustSensitivity(double power);
  double applyDeadband(double power);
  double capAcceleration(double targetPower, double curPower, double maxAcceleration);
};
