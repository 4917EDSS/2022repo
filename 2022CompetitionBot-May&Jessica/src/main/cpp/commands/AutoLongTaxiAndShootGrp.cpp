// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoLongTaxiAndShootGrp.h"
#include "commands/DriveStraightCmd.h"
#include "commands/RotateRobotCmd.h"
#include "commands/ShootCargoCmd.h"
#include "commands/AlignThenShootGrp.h"

// Required Initial Start State:
// - Location:  Any tarmac location where cargo and guardrails won't be in the way while backing up 2m.  
//              Same action as "ShortTaxiAndShootGrp" except with 2m reverse instead of 1.2m reverse.
// - Orientation:  Facing the hub
// - Cargo:  One cargo preloaded from the top, pressed up against the front of the magazine
AutoLongTaxiAndShootGrp::AutoLongTaxiAndShootGrp(DrivetrainSub *drivetrainSub, ShooterSub *shooterSub, VisionSub *visionSub, IntakeSub *intakeSub) {
  // Add your commands here, e.g.
   //AddCommands(FooCommand(), BarCommand());

  AddCommands(
    DriveStraightCmd(drivetrainSub, -2),
    AlignThenShootGrp(shooterSub, visionSub, drivetrainSub, intakeSub, true)
  ); 
}
