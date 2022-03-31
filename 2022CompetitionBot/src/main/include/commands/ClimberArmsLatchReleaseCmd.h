// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSub.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimberArmsLatchReleaseCmd
    : public frc2::CommandHelper<frc2::CommandBase, ClimberArmsLatchReleaseCmd> {
 public:
  ClimberArmsLatchReleaseCmd(ClimberSub * climberSub, bool isFoldIn,  bool isDelayAtStart, bool isDelayAtEnd);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ClimberSub * m_climberSubPtr;
  bool m_isFoldIn;
  bool m_executeCmd;
  bool m_hasExecuted;
  bool m_isDelayAtStart;
  bool m_isDelayAtEnd;
  uint64_t m_startTime;
};
