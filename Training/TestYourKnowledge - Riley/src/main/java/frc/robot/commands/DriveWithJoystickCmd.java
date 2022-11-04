// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj.PS4Controller;

public class DriveWithJoystickCmd extends CommandBase {
  RomiDrivetrain m_drivetrainSub;
  PS4Controller m_controller;

  public DriveWithJoystickCmd(PS4Controller controller, RomiDrivetrain drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    m_controller = controller;
    addRequirements(drivetrainSub);
  }
  

  @Override
  public void execute() {
    m_drivetrainSub.arcadeDrive(m_controller.getLeftY(), m_controller.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.arcadeDrive(0.0, 0.0);
  }
}
