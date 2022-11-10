// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new SwerveDrivetrain. */

  // Steering Encoders

  private static final double fl_encoderOffset = 0.0;
  private static final double fr_encoderOffset = 0.0;
  private static final double bl_encoderOffset = 0.0;
  private static final double br_encoderOffset = 0.0;
  
  //private final CANCoderConfiguration m_CANConfig; // Configuration settings for encoders

  private final CANCoder m_encoderFrontLeft = new CANCoder(Constants.CanIds.kEncoderFL);

  private final CANCoder m_encoderFrontRight = new CANCoder(Constants.CanIds.kEncoderFR);

  private final CANCoder m_encoderBackLeft = new CANCoder(Constants.CanIds.kEncoderBL);

  private final CANCoder m_encoderBackRight = new CANCoder(Constants.CanIds.kEncoderBR);


  // Motors

  private final CANSparkMax m_frontleftSteerMotor = new CANSparkMax(Constants.CanIds.kSteeringMotorFL,CANSparkMax.MotorType.kBrushless);
  private final TalonFX m_frontleftDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorFL);

  private final CANSparkMax m_frontrightSteerMotor = new CANSparkMax(Constants.CanIds.kSteeringMotorFR,CANSparkMax.MotorType.kBrushless);
  private final TalonFX m_frontrightDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorFR);

  private final CANSparkMax m_backleftSteerMotor = new CANSparkMax(Constants.CanIds.kSteeringMotorBL,CANSparkMax.MotorType.kBrushless);
  private final TalonFX m_backleftDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorBL);

  private final CANSparkMax m_backrightSteerMotor = new CANSparkMax(Constants.CanIds.kSteeringMotorBR,CANSparkMax.MotorType.kBrushless);
  private final TalonFX m_backrightDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorBR);
  public SwerveDrivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Display Encoder angles and velocities
    SmartDashboard.putNumber("Front Left Angle (ABS): ",m_encoderFrontLeft.getAbsolutePosition());
    SmartDashboard.putNumber("Front Right Angle (ABS): ",m_encoderFrontRight.getAbsolutePosition());
    SmartDashboard.putNumber("Back Left Angle (ABS): ",m_encoderBackLeft.getAbsolutePosition());
    SmartDashboard.putNumber("Back Right Angle (ABS): ",m_encoderBackRight.getAbsolutePosition());

    SmartDashboard.putNumber("Front Left ID: ",m_encoderFrontLeft.getDeviceID());
    SmartDashboard.putNumber("Front Right ID: ",m_encoderFrontRight.getDeviceID());
    SmartDashboard.putNumber("Back Left ID: ",m_encoderBackLeft.getDeviceID());
    SmartDashboard.putNumber("Back Right ID: ",m_encoderBackRight.getDeviceID());
    
  }

  // Steering functions

  public double getAngle(int id) { // Get Steering encoder (0: FL, 1: FR, 2: BL, 3: BR)
    double angle = 0.0;
    switch(id) {
      case 0:

      break;

      case 1:
      
      break;

      case 2:
      
      break;

      case 3:
      
      break;

    }
    return angle;
  }
  public double getGlobalAngle(int id) { // Get Steering encoder (0: FL, 1: FR, 2: BL, 3: BR)
    double angle = 0.0;
    switch(id) {
      case 0:

      break;

      case 1:
      
      break;

      case 2:
      
      break;

      case 3:
      
      break;

    }
    return angle;
  }

  public void motorDrive(double power) {
    /*m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,power); // Uncomment to spin drive motors
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,power);
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,power);
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,power);*/

    
    m_frontleftSteerMotor.set(power); // Uncomment to spin steering motors
    m_frontrightSteerMotor.set(power);
    m_backleftSteerMotor.set(power);
    m_backrightSteerMotor.set(power);
  }
  public void driveMotor(int motor, double power) { // Spin a drive motor (FL,FR,BL,BR)
    switch(motor) {
      case 0:
      m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,power);
      break;
      case 1:
      m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,power);
      break;
      case 2:
      m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,power);
      break;
      case 3:
      m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,power);
      break;
    }
  }
  public void steerMotor(int motor, double power) { // Spin a steering motor (FL,FR,BL,BR)
    switch(motor) {
      case 0:
      m_frontleftSteerMotor.set(power);
      break;
      case 1:
      m_frontrightSteerMotor.set(power);
      break;
      case 2:
      m_backleftSteerMotor.set(power);
      break;
      case 3:
      m_backrightSteerMotor.set(power);
      break;
    }
  }
  public void brake() { // Set all motors to 0 power
    m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);

    
    m_frontleftSteerMotor.set(0.0);
    m_frontrightSteerMotor.set(0.0);
    m_backleftSteerMotor.set(0.0);
    m_backrightSteerMotor.set(0.0);
  }

  
}
