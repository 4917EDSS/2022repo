// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberArmCmd.h"

constexpr double kClimberArmPower = 0.1;

ClimberArmCmd::ClimberArmCmd(ClimberSub* climberSub, bool climberDirection) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({climberSub});
  m_climberSubPtr = climberSub;
  m_climberDirection = climberDirection;
}

// Called when the command is initially scheduled.
void ClimberArmCmd::Initialize() {
  if (m_climberDirection){
    m_climberSubPtr-> setClimberArmPower(-kClimberArmPower);
  } else {
    m_climberSubPtr-> setClimberArmPower(kClimberArmPower);
  }

}

// Called repeatedly when this Command is scheduled to run
void ClimberArmCmd::Execute() {}

// Called once the command ends or is interrupted.
void ClimberArmCmd::End(bool interrupted) {
  m_climberSubPtr-> setClimberArmPower(0.);
}

// Returns true when the command should end.
bool ClimberArmCmd::IsFinished() {
  return false;
}
