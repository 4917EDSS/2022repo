// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSub.h"
#include "subsystems/DrivetrainSub.h"
#include "subsystems/IntakeSub.h"
#include "subsystems/ShooterSub.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class KillEverythingCmd
    : public frc2::CommandHelper<frc2::CommandBase, KillEverythingCmd> {
 public:
  KillEverythingCmd(ClimberSub* climberSub, DrivetrainSub* driveTrainSub, IntakeSub* intakeSub, ShooterSub* shooterSub);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};
