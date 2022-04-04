// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCargoCmd.h"
#include <frc/RobotController.h>

constexpr uint64_t stop_time = 3000000; //3 seconds

IntakeCargoCmd::IntakeCargoCmd(IntakeSub* intakeSub) {
  IntakeCargoCmd(intakeSub, false);
}

//Timeout override
IntakeCargoCmd::IntakeCargoCmd(IntakeSub* intakeSub, bool timeout) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intakeSub});
  m_intakeSubPtr = intakeSub;

  m_doTimeout = timeout;
}


// Called when the command is initially scheduled.
void IntakeCargoCmd::Initialize() {
  m_startTime = frc::RobotController::GetFPGATime();

  if(!m_intakeSubPtr->isCargoAtMagazineBack()) {
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
  }
  if(m_intakeSubPtr->isCargoAtMagazineFront() && !m_intakeSubPtr->isCargoAtMagazineBack()) {
    m_intakeSubPtr->enableMagazineMotor(false);
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
  } else if (m_doTimeout && (frc::RobotController::GetFPGATime()-m_startTime) > stop_time){
    return true; 
  }
  else {
    return false;
  }
}
