// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

class VisionSub : public frc2::SubsystemBase {
 public:
  VisionSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  //Get camera code
  std::shared_ptr<nt::NetworkTable> getTable() {return nt::NetworkTableInstance::GetDefault().GetTable("limelight"); }

  void Periodic() override;


  //Get raw values for stuff
  double getHorizontalAngle() {return getTable()->GetNumber("tx",0.0); }
  double getVerticalAngle() {return getTable()->GetNumber("ty",0.0); }
  double getTargetArea() {return getTable()->GetNumber("ta",0.0); }
  double getTargetSkew() {return getTable()->GetNumber("ts",0.0); }

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};