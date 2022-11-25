// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DrivetrainSub;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.PS4Controller;

public class DrivewithjoystickCmd extends CommandBase {
  /** Creates a new DriveCmd. */
  private DrivetrainSub m_drivetrainSub; 
  private PS4Controller m_controller;

  public DrivewithjoystickCmd(PS4Controller controller, DrivetrainSub drivetrainSub){
    m_drivetrainSub = drivetrainSub;
     m_controller = controller;

    addRequirements(drivetrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSub.tankDrive(m_controller.getLeftY(), m_controller.getRightX());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
