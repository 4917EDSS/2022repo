// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbStageOneGrp.h"
#include "commands/ClimberArmRaiseCmd.h"
#include "commands/ClimberArmLowerCmd.h"
#include "commands/ClimberArmsLatchReleaseCmd.h"

ClimbStageOneGrp::ClimbStageOneGrp(ClimberSub *climberSub) {
  //Release arm, lower wench, latch arm, raise wench
  AddCommands(
    ClimberArmsLatchReleaseCmd(climberSub,false),
    ClimberArmLowerCmd(climberSub),
    ClimberArmsLatchReleaseCmd(climberSub, true),
    ClimberArmRaiseCmd(climberSub)
  );
}

