// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCargoCmd.h"

IntakeCargoCmd::IntakeCargoCmd(IntakeSub* intakeSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intakeSub});
  m_intakeSubPtr = intakeSub;
}

// Called when the command is initially scheduled.
void IntakeCargoCmd::Initialize() {
  m_intakeSubPtr->lowerIntake();
  m_intakeSubPtr->enableFrontRollerIntakeMotor(false);
}

// Called repeatedly when this Command is scheduled to run
void IntakeCargoCmd::Execute() {
  if ((m_intakeSubPtr->isCargoAtIntakeEnd()||m_intakeSubPtr->isCargoAtMagazineFront()) && !m_intakeSubPtr->isCargoAtMagazineBack()) {
    m_intakeSubPtr->enableMagazineMotor(false);
  } else {
    m_intakeSubPtr->disableMagazineMotor();
  }
}

// Called once the command ends or is interrupted.
void IntakeCargoCmd::End(bool interrupted) {
  m_intakeSubPtr->raiseIntake();
  m_intakeSubPtr->disableFrontRollerIntakeMotor();
  m_intakeSubPtr->disableMagazineMotor();
}

// Returns true when the command should end.
bool IntakeCargoCmd::IsFinished() {
  if (m_intakeSubPtr->isCargoAtMagazineBack()) {
    return true;
  } else {
    return false; 
  }
}
