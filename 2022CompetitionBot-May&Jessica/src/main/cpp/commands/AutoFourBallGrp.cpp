// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoFourBallGrp.h"
#include "commands/DriveStraightCmd.h"
#include "commands/ShootCargoCmd.h"
#include "commands/AlignThenShootGrp.h"
#include "commands/IntakeCargoCmd.h" 
#include "commands/RotateRobotCmd.h"
#include <frc2/command/ParallelCommandGroup.h>

// Required Initial Start State:
// - Location:  ?
// - Orientation:  Facing aways from the hub and lined up with the cargo
// - Cargo:  One cargo preloaded from the top, pressed up against the front of the magazine

AutoFourBallGrp::AutoFourBallGrp(ShooterSub* shooterSub, IntakeSub* intakeSub, DrivetrainSub* drivetrainSub, VisionSub* visionSub) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands( 
    //Retrieve and shoot first ball
    frc2::ParallelCommandGroup {
      DriveStraightCmd(drivetrainSub, 1.9), 
      IntakeCargoCmd(intakeSub)
    },
    RotateRobotCmd(drivetrainSub, -180), 
    AlignThenShootGrp(shooterSub, visionSub, drivetrainSub, intakeSub, true),
    RotateRobotCmd(drivetrainSub, -149),
    frc2::ParallelCommandGroup {
      DriveStraightCmd(drivetrainSub, 3.5), 
      IntakeCargoCmd(intakeSub)
    },
    RotateRobotCmd(drivetrainSub, 160),
    DriveStraightCmd(drivetrainSub, 3.0),
    AlignThenShootGrp(shooterSub, visionSub, drivetrainSub, intakeSub, true)
  );
} 

