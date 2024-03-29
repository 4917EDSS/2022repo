// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class DriveWithJoystickCmd extends CommandBase {
  /** Creates a new DriveWithJoystickCmd. */
  private DrivetrainSub m_drivetrainSub;
  private PS4Controller m_controller;

  public DriveWithJoystickCmd(DrivetrainSub drivetrainSub, PS4Controller controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSub = drivetrainSub;
    m_controller = controller;
  
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSub.tankDrive(m_controller.getLeftY(), m_controller.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
