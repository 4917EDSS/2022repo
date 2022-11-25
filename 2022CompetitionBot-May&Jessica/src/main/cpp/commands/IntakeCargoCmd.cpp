// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCargoCmd.h"
#include <frc/RobotController.h>
#include <iostream>

IntakeCargoCmd::IntakeCargoCmd(IntakeSub* intakeSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intakeSub});
  m_intakeSubPtr = intakeSub;
}

// Called when the command is initially scheduled.
void IntakeCargoCmd::Initialize() {
  if(!m_intakeSubPtr->isCargoAtMagazineBack()) {
    //std::cout << "initalize and cargo in back magazine" << frc::RobotController::GetFPGATime() << std::endl;
    m_intakeSubPtr->lowerIntake();
    m_intakeSubPtr->enableFrontRollerIntakeMotor(false);
  }
  atCargoMagazineFront = false;
}

// Called repeatedly when this Command is scheduled to run

void IntakeCargoCmd::Execute() {
  // We want to start the rollers the second there is a ball at intake end.
  // We want to continue the rollers unconditionally, until we see a ball at either the magazine front or back.
  // If we see a ball at the magazine back, we're done. Turn off the motors.
  // If we see a ball at the magazine front, we are going to continue magazine until we can't see it.

  if(m_intakeSubPtr->isCargoAtIntakeEnd() && !m_intakeSubPtr->isCargoAtMagazineBack()) {
    m_intakeSubPtr->enableMagazineMotor(false);
    //std::cout << "cargo at intake end" << frc::RobotController::GetFPGATime() << std::endl;
  }
  if(m_intakeSubPtr->isCargoAtMagazineFront() && !m_intakeSubPtr->isCargoAtMagazineBack()) {
    m_intakeSubPtr->enableMagazineMotor(false);
    //std::cout << "cargo at magazine front" << frc::RobotController::GetFPGATime() << std::endl;
    atCargoMagazineFront = true;
  }
  if(atCargoMagazineFront){
    if(!m_intakeSubPtr->isCargoAtMagazineFront()) {
      m_intakeSubPtr->disableMagazineMotor();
      atCargoMagazineFront = false;
    }
  }
  if(m_intakeSubPtr->isCargoAtMagazineBack()){
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
