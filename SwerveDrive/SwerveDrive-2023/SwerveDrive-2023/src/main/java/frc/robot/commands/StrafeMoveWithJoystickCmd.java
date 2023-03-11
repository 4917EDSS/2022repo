// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class StrafeMoveWithJoystickCmd extends CommandBase {
  /** Creates a new StrafeMoveWithJoystick. */

  SwerveDrivetrain m_swerveDrivetrainSub;
  PS4Controller m_controller;

  public StrafeMoveWithJoystickCmd(SwerveDrivetrain swerveSub, PS4Controller controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDrivetrainSub = swerveSub;
    m_controller = controller;
    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnPower = m_controller.getLeftX();
    double fwdPower = m_controller.getRawAxis(5);

    double power = 0.0;

    // Get facing direction
    if(m_controller.getSquareButton()) {
      if(m_controller.getRawAxis(Constants.R2Axis) > 0.1) { // Forwards
        power = m_controller.getRawAxis(Constants.R2Axis);
      } else if(m_controller.getRawAxis(Constants.L2Axis) > 0.1) { //Backwards
        power = -m_controller.getRawAxis(Constants.L2Axis);
      } else {
        power = 0.0;
      }
      m_swerveDrivetrainSub.circularDrive(power);
    } else if(m_controller.getCrossButton()) {
      m_swerveDrivetrainSub.setSteeringAngle(0.0);
    } else {
      m_swerveDrivetrainSub.arcadeSwerveDemo(fwdPower, turnPower);
    }

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
