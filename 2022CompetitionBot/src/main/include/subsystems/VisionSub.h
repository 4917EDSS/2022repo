// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/Shuffleboard.h>


namespace VisionConstants {
  constexpr double kMountAngleDegrees = 21.0; //Degrees that camera is rotated back from vertical
  constexpr double kLensHeightInches = 32.5; //Distance from floor to center of camera in inches
  constexpr double kGoalHeightInches = 95.0; //Will need to be changed (104.0)
}

class VisionSub : public frc2::SubsystemBase {
 public:
  VisionSub();
  void init();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  //Get camera code
  std::shared_ptr<nt::NetworkTable> getTable() {return nt::NetworkTableInstance::GetDefault().GetTable("limelight"); }

  void Periodic() override;


  //Get raw values for stuff
  double getHorizontalAngle() {return getTable()->GetNumber("tx",0.0); }//Resolution of limelight is 320x240
  double getVerticalAngle() {return getTable()->GetNumber("ty",0.0); }
  double getTargetArea() {return getTable()->GetNumber("ta",0.0); }
  double getTargetSkew() {return getTable()->GetNumber("ts",0.0); }
  bool isValidTarget() {return getTable()->GetNumber("tv",0.0); } //(1) if any valid targets (0) If none

  double estimateDistanceMeters();
  
  void  targetVisionPipeline();
  void  targetNeutralVisionPipeline();
 private:
  
  nt::NetworkTableEntry distEntry;
  nt::NetworkTableEntry angleEntry;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
