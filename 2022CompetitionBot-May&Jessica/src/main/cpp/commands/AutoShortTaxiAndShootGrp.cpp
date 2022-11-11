// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoShortTaxiAndShootGrp.h"
#include "commands/DriveStraightCmd.h"
#include "commands/ShootCargoCmd.h"
#include "commands/AlignThenShootGrp.h"

// Required Initial Start State:
// - Location:  Any tarmac location where cargo and guardrails could be in the way or where backing up the longer 2m would be too far from the hub.
//              Same action as "LongTaxiAndShootGrp" except with 2m reverse instead of 1.2m reverse.
// - Orientation:  Facing the hub
// - Cargo:  One cargo preloaded from the top, pressed up against the front of the magazine

AutoShortTaxiAndShootGrp::AutoShortTaxiAndShootGrp(ShooterSub* shooterSub, IntakeSub* intakeSub, DrivetrainSub* drivetrainSub,VisionSub* visionSub) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
    DriveStraightCmd(drivetrainSub, -1.2), 
    AlignThenShootGrp(shooterSub, visionSub, drivetrainSub, intakeSub, true)
  );
}
