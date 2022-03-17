// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TwoBallAutoGrp.h"
#include "commands/DriveStraightCmd.h"
#include "commands/IntakeCargoCmd.h"
#include "commands/RotateRobotCmd.h"
#include "commands/ShootCargoCmd.h"
#include <frc2/command/ParallelCommandGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoBallAutoGrp::TwoBallAutoGrp(DrivetrainSub *drivetrainSub, IntakeSub *intakeSub, ShooterSub *shooterSub, VisionSub *visionSub) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());

  AddCommands(
    //Retrieve and shoot first ball
    frc2::ParallelCommandGroup{DriveStraightCmd ( drivetrainSub, 2), IntakeCargoCmd(intakeSub)},
    RotateRobotCmd(drivetrainSub, -180) 
    // ShootCargoCmd(shooterSub, intakeSub, visionSub, true)
    //Drive to center line
    //RotateRobotCmd(drivetrainSub, -45),
    //DriveStraightCmd(drivetrainSub, 1)
  );
}
