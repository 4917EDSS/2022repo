// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardCmd extends CommandBase {
  private final RomiDrivetrain m_drivetrainSub;
  
  public DriveForwardCmd(RomiDrivetrain drivetrainSub) {
    addRequirements(drivetrainSub);
    
    m_drivetrainSub = drivetrainSub;
  }//#endregion)
  /** Creates a new DriveForwardCmd. */
  
    // Use addRequirements() here to declare subsystem dependencies.
  


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.arcadeDrive(0.5, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
