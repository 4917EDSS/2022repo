// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DrivetrainSub.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RotateRobotCmd
    : public frc2::CommandHelper<frc2::CommandBase, RotateRobotCmd> {
 public:
  RotateRobotCmd(DrivetrainSub *drivetrainSub, double angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  DrivetrainSub *m_drivetrainSubPtr;
  double m_angle;
  double power;
};
