/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {
  public final static class CanIds {
    public final static int kLeftMotor1 = 1;
    public final static int kRightMotor1 = 4;
  }

  private final CANSparkMax m_leftMotor1 =
     new CANSparkMax(Constants.CanIds.kLeftMotor1,
     CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rightMotor1 =
     new CANSparkMax(Constants.CanIds.kRightMotor1,
     CANSparkMaxLowLevel.MotorType.kBrushless);
  
  public void tankDrive(double leftPower, double rightPower){
    m_leftMotor1.set(leftPower);
    m_rightMotor1.set(rightPower);
  }
     /**
   * Creates a new DrivetrainSub.
   */
  public DrivetrainSub() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
