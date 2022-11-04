// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class DriveForwardCmd extends CommandBase {
  private final DrivetrainSub m_drivetrainSub;
  /** Creates a new DriveForwardCmd. */
  public DriveForwardCmd(DrivetrainSub drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
    
    m_drivetrainSub = drivetrainSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSub.tankDrive(0.50, 0.0);}
 
  
 // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
