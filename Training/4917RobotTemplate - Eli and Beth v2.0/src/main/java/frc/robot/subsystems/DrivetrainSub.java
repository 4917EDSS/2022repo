// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSub;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class DrivetrainSub extends SubsystemBase {
  private final CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.CanIds.kLeftMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(Constants.CanIds.kLeftMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_leftMotor3 = new CANSparkMax(Constants.CanIds.kLeftMotor3, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.CanIds.kRightMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(Constants.CanIds.kRightMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor3 = new CANSparkMax(Constants.CanIds.kRightMotor3, CANSparkMaxLowLevel.MotorType.kBrushless);

  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void tankDrive(double leftPower, double rightPower) {
    m_leftMotor1.set(leftPower);
    m_leftMotor2.set(leftPower);
    m_leftMotor3.set(leftPower);
    m_rightMotor1.set(rightPower);
    m_rightMotor2.set(rightPower);
    m_rightMotor3.set(rightPower);
  }

public void arcadeDrive(double d, double e) {
}
}
