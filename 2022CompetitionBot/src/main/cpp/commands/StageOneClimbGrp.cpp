// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StageOneClimbGrp.h"
#include "commands/RaiseClimberArmCmd.h"
#include "commands/LowerClimberArmCmd.h"
#include "commands/SetClimberArmCmd.h"

StageOneClimbGrp::StageOneClimbGrp(ClimberSub *climberSub) {
  //Release arm, lower wench, latch arm, raise wench
  AddCommands(SetClimberArmCmd(climberSub,false),LowerClimberArmCmd(climberSub),SetClimberArmCmd(climberSub, true), RaiseClimberArmCmd(climberSub));
}

