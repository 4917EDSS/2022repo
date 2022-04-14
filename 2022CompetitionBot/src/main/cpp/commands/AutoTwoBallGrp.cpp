// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoTwoBallAutoGrp.h"
#include "commands/DriveStraightCmd.h"
#include "commands/IntakeCargoCmd.h"
#include "commands/RotateRobotCmd.h"
#include "commands/ShootCargoCmd.h"
#include "commands/AlignThenShootGrp.h"
#include <frc2/command/ParallelCommandGroup.h>

// Required Initial Start State:
// - Location:  Left-tarmac (facing our hanger) lined up with our cargo, or
//              Right-tarmac (facing our terminal) lined up with cargoa
// - Orientation:  Facing our cargo and facing away from the hub
// - Cargo:  One cargo preloaded from the top, pressed up against the front of the magazine

AutoTwoBallGrp::AutoTwoBallGrp(DrivetrainSub *drivetrainSub, IntakeSub *intakeSub, ShooterSub *shooterSub, VisionSub *visionSub) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());

  AddCommands(
    //Retrieve and shoot first ball
    frc2::ParallelCommandGroup{
      DriveStraightCmd(drivetrainSub, 1.9), 
      IntakeCargoCmd(intakeSub)},
    RotateRobotCmd(drivetrainSub, -180), 
    AlignThenShootGrp(shooterSub, visionSub, drivetrainSub, intakeSub, true)
  );
}
