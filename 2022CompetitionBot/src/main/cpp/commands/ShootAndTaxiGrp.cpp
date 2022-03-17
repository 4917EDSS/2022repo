// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootAndTaxiGrp.h"
#include "commands/DriveStraightCmd.h"
#include "commands/ShootCargoCmd.h"
#include "commands/AlignThenShootGrp.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ShootAndTaxiGrp::ShootAndTaxiGrp(ShooterSub* shooterSub, IntakeSub* intakeSub, DrivetrainSub* drivetrainSub,VisionSub* visionSub) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands( DriveStraightCmd(drivetrainSub, -2), AlignThenShootGrp(shooterSub, visionSub, drivetrainSub, intakeSub));
}
