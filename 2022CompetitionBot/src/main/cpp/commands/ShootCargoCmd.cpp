// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/RobotController.h>
#include "commands/ShootCargoCmd.h"
#include "iostream"

constexpr double kDistanceMin=1.5;
constexpr double kDistanceMax=6.5;
constexpr double kSpeedMin=12000.0;
constexpr double kSpeedMax=26400.0;

ShootCargoCmd::ShootCargoCmd(ShooterSub* shooterSub, IntakeSub* intakeSub, VisionSub *visionSub, bool isUpperGoal, bool isAuto) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooterSub});
  AddRequirements({intakeSub});
  AddRequirements({visionSub});

  m_shooterSubPtr = shooterSub;
  m_visionSubPtr = visionSub;
  m_intakeSubPtr = intakeSub;
  m_isUpperGoal = isUpperGoal;
  m_isAuto = isAuto;
}


// Called when the command is initially scheduled.
void ShootCargoCmd::Initialize() {
  m_visionSubPtr->targetVisionPipeline();
  m_ballLastSeenTime = frc::RobotController::GetFPGATime();
  m_isUpToSpeed=false;
  if(m_isUpperGoal) {
    if (m_isAuto == true) { 
    m_intakeSubPtr->lowerIntake();
    m_intakeSubPtr->enableFrontRollerIntakeMotor(false);
    }
    // y is speed, x is distance (one least and two greatest) y = mx+b **assumes linear relationship
    double currentDistance = m_visionSubPtr->estimateDistanceMeters();
    double slope=(kSpeedMax-kSpeedMin)/(kDistanceMax-kDistanceMin);
    double intercept=kSpeedMin-(slope*kDistanceMin);
    if (currentDistance == 0.0) {
      m_targetSpeed = m_shooterSubPtr->m_upperBinSpeed;
    } else {
      m_targetSpeed=(slope*currentDistance)+intercept;
    }
  }
  else {
    m_targetSpeed = m_shooterSubPtr->m_lowerBinSpeed;
  }

  m_shooterSubPtr->autoVelocity(m_targetSpeed);
}

// Called repeatedly when this Command is scheduled to run
void ShootCargoCmd::Execute() {
  if(fabs(m_targetSpeed - m_shooterSubPtr->getSpeed()) < ShooterConstants::kShootTolerance) {
    m_isUpToSpeed=true;
  }
  if (m_isUpToSpeed) {
    m_intakeSubPtr->setMagazineMotor(0.4);
  }
  else {
    m_intakeSubPtr->disableMagazineMotor();
  }
  if (m_intakeSubPtr->isCargoAtMagazineBack()){
    m_ballLastSeenTime = frc::RobotController::GetFPGATime();
  }
}

// Called once the command ends or is interrupted.
void ShootCargoCmd::End(bool interrupted) {
  m_visionSubPtr->targetNeutralVisionPipeline();
  m_shooterSubPtr->setPower(0);
  m_intakeSubPtr->disableFrontRollerIntakeMotor();
  m_intakeSubPtr->raiseIntake();
  m_intakeSubPtr->disableMagazineMotor();
}

// Returns true when the command should end.
bool ShootCargoCmd::IsFinished() {
  if(frc::RobotController::GetFPGATime() - m_ballLastSeenTime > 3000000){
    return true;
  }
  return false;
}