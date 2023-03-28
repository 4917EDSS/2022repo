// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vector;
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
    double rightX = m_controller.getRawAxis(4);
    double rightY = m_controller.getRawAxis(5);
    double fwdPower = Math.sqrt(rightX * rightX + rightY * rightY);

    Vector stuff = new Vector(rightX, rightY);
    m_swerveDrivetrainSub.trueDrive(stuff, turnPower);
    if(m_controller.getSquareButton()) {
      m_swerveDrivetrainSub.setSteeringAngle(0.0);
    } else if(m_controller.getCircleButton()) {
      m_swerveDrivetrainSub.resetGyro();
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
