// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoLongTaxiAndShootGrp.h"
#include "commands/DriveStraightCmd.h"
#include "commands/RotateRobotCmd.h"
#include "commands/ShootCargoCmd.h"
#include "commands/AlignThenShootGrp.h"

AutoLongTaxiAndShootGrp::AutoLongTaxiAndShootGrp(DrivetrainSub *drivetrainSub, ShooterSub *shooterSub, VisionSub *visionSub, IntakeSub *intakeSub) {
  // Add your commands here, e.g.
   //AddCommands(FooCommand(), BarCommand());

  AddCommands(
    DriveStraightCmd(drivetrainSub, -2),
    AlignThenShootGrp(shooterSub, visionSub, drivetrainSub, intakeSub, true)
  ); 
}
