// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootAndTaxiGrp.h"
#include "commands/DriveStraightCmd.h"
#include "commands/ShootCargoCmd.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ShootAndTaxiGrp::ShootAndTaxiGrp(ShooterSub* shooterSub, IntakeSub* intakeSub, DrivetrainSub* drivetrainSub) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(ShootCargoCmd(shooterSub, intakeSub, true), DriveStraightCmd(drivetrainSub, 3));//Drives 3 meters
}
