// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSub.h"
#include "subsystems/IntakeSub.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootCargoCmd
    : public frc2::CommandHelper<frc2::CommandBase, ShootCargoCmd> {
 public:
  ShootCargoCmd(ShooterSub* shooterSub, IntakeSub* intakeSub, bool isUpperGoal);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  ShooterSub *m_shooterSubPtr;
  IntakeSub *m_intakeSubPtr;

  int m_isUpperGoal;
  double m_targetSpeed;
};
