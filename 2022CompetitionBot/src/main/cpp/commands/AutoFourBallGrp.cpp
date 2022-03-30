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

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AutoFourBallGrp::AutoFourBallGrp(ShooterSub* shooterSub, IntakeSub* intakeSub, DrivetrainSub* drivetrainSub, VisionSub* visionSub) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands( 
    //Retrieve and shoot first ball
    frc2::ParallelCommandGroup{DriveStraightCmd(drivetrainSub, 2), IntakeCargoCmd(intakeSub)},
    RotateRobotCmd(drivetrainSub, -180), 
    AlignThenShootGrp(shooterSub, visionSub, drivetrainSub, intakeSub, true),
    RotateRobotCmd(drivetrainSub, -150),
    frc2::ParallelCommandGroup{DriveStraightCmd(drivetrainSub, 2.5), IntakeCargoCmd(intakeSub)},
    RotateRobotCmd(drivetrainSub, 150),
    DriveStraightCmd(drivetrainSub, 2.5),
    AlignThenShootGrp(shooterSub, visionSub, drivetrainSub, intakeSub, true)
  );
}

