// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/ShooterSub.h"
#include "subsystems/VisionSub.h"
#include "subsystems/DrivetrainSub.h"
#include "subsystems/IntakeSub.h"

class AlignThenShootGrp
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, AlignThenShootGrp> {
 public:
  AlignThenShootGrp(ShooterSub *shooterSub,VisionSub *visionSub, DrivetrainSub *drivetrainSub,IntakeSub *intakeSub);
};
