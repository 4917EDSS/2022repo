// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

  public class DrivetrainSub extends CommandBase {
  private final CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.CanIds.kLeftMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.CanIds.kRightMotor1,CANSparkMaxLowLevel.MotorType.kBrushless);

  public void 	tankDrive(double leftPower, double rightPower) {
  m_leftMotor1.set(leftPower);
  m_rightMotor1.set(rightPower);
}

  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
