// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj.PS4Controller;

public class DriveJoystickCmd extends CommandBase {
  RomiDrivetrain m_romiDrivetrain;
  PS4Controller m_controller;

  /** Creates a new DriveJoystickCmd. */
  public DriveJoystickCmd(PS4Controller controller, RomiDrivetrain romidrivetrain) {
    m_romiDrivetrain = romidrivetrain;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(romidrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Normally we would put this in initialize but the FRC differential drive object
    // times out if you don't call arcadeDrive() continuously.
    m_romiDrivetrain.arcadeDrive(m_controller.getLeftY(), m_controller.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_romiDrivetrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
