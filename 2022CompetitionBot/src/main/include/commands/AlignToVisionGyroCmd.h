// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/DrivetrainSub.h>
#include <subsystems/VisionSub.h>
#include <commands/DriveWithJoystickCmd.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AlignToVisionGyroCmd
    : public frc2::CommandHelper<frc2::CommandBase, AlignToVisionGyroCmd> {
 public:
  AlignToVisionGyroCmd(DrivetrainSub *drivetrainSub, VisionSub *visionSub);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DrivetrainSub *m_drivetrainSubPtr;
  VisionSub *m_visionSubPtr;
  DriveWithJoystickCmd *m_driveWithJoystickCmd;
  uint64_t m_startTime;

  double m_angle;
};
