// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class DriveForwardCmd extends CommandBase {
  /** Creates a new DriveForwardCmd. */
  private final DrivetrainSub m_drivetrainSub;

  public DriveForwardCmd(DrivetrainSub drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSub = drivetrainSub;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Drive forward at 25% power
    m_drivetrainSub.tankDrive(0.25, 0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
