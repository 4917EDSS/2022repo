// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

import java.util.ArrayList;


public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new SwerveDrivetrain. */

  private static final Orchestra orcs = new Orchestra();
  // Steering Encoders

  private double kP = 1.0;
  private double kI = 1.0;
  private double kD = 1.0;

  private static final double fl_encoderOffset = 215.1;
  private static final double fr_encoderOffset = 210.1;
  private static final double bl_encoderOffset = 182.3;
  private static final double br_encoderOffset = 82.7;
  
  //private final CANCoderConfiguration m_CANConfig; // Configuration settings for encoders

  private final CANCoder m_encoderFrontLeft = new CANCoder(Constants.CanIds.kEncoderFL);

  private final CANCoder m_encoderFrontRight = new CANCoder(Constants.CanIds.kEncoderFR);

  private final CANCoder m_encoderBackLeft = new CANCoder(Constants.CanIds.kEncoderBL);

  private final CANCoder m_encoderBackRight = new CANCoder(Constants.CanIds.kEncoderBR);


  // Motors

  private final CANSparkMax m_frontleftSteerMotor = new CANSparkMax(Constants.CanIds.kSteeringMotorFL,CANSparkMax.MotorType.kBrushless);
  private static final TalonFX m_frontleftDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorFL);

  private final CANSparkMax m_frontrightSteerMotor = new CANSparkMax(Constants.CanIds.kSteeringMotorFR,CANSparkMax.MotorType.kBrushless);
  private static final TalonFX m_frontrightDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorFR);

  private final CANSparkMax m_backleftSteerMotor = new CANSparkMax(Constants.CanIds.kSteeringMotorBL,CANSparkMax.MotorType.kBrushless);
  private static final TalonFX m_backleftDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorBL);

  private final CANSparkMax m_backrightSteerMotor = new CANSparkMax(Constants.CanIds.kSteeringMotorBR,CANSparkMax.MotorType.kBrushless);
  private static final TalonFX m_backrightDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorBR);

  private final PIDController pid = new PIDController(0.1, 0, 0.0);
  public SwerveDrivetrain() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Display Encoder angles and velocities
    SmartDashboard.putNumber("Front Left Angle (ABS): ", m_encoderFrontLeft.getAbsolutePosition()-fl_encoderOffset);
    SmartDashboard.putNumber("Front Right Angle (ABS): ",m_encoderFrontRight.getAbsolutePosition()-fr_encoderOffset);
    SmartDashboard.putNumber("Back Left Angle (ABS): ",m_encoderBackLeft.getAbsolutePosition()-bl_encoderOffset);
    SmartDashboard.putNumber("Back Right Angle (ABS): ",m_encoderBackRight.getAbsolutePosition()-br_encoderOffset);

    SmartDashboard.putNumber("Front Left ID: ",m_encoderFrontLeft.getDeviceID());
    SmartDashboard.putNumber("Front Right ID: ",m_encoderFrontRight.getDeviceID());
    SmartDashboard.putNumber("Back Left ID: ",m_encoderBackLeft.getDeviceID());
    SmartDashboard.putNumber("Back Right ID: ",m_encoderBackRight.getDeviceID());

    

    kP = SmartDashboard.getNumber("kP", 1.0);
    kI = SmartDashboard.getNumber("kI", 0.0);
    kD = SmartDashboard.getNumber("kD", 0.0);

    SmartDashboard.putNumber("kP",kP);
    SmartDashboard.putNumber("kI",kI);
    SmartDashboard.putNumber("kD",kD);
    
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
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
  public double vecToAngle(double x, double y) {
    return Math.atan2(x,y) * 180.0f / 3.14159;
  }

  public void setAngle(double angle) {
    pid.enableContinuousInput(0, 180);
    double powerFL = MathUtil.clamp(pid.calculate(m_encoderFrontLeft.getAbsolutePosition()-fl_encoderOffset,angle%180),-0.3,0.3);
    double powerFR = MathUtil.clamp(pid.calculate(m_encoderFrontRight.getAbsolutePosition()-fr_encoderOffset,angle%180),-0.3,0.3);
    double powerBL = MathUtil.clamp(pid.calculate(m_encoderBackLeft.getAbsolutePosition()-bl_encoderOffset,angle%180),-0.3,0.3);
    double powerBR = MathUtil.clamp(pid.calculate(m_encoderBackRight.getAbsolutePosition()-br_encoderOffset,angle%180),-0.3,0.3);
    m_frontleftSteerMotor.set(powerFL);
    m_frontrightSteerMotor.set(powerFR);
    m_backleftSteerMotor.set(powerBL);
    //m_backrightSteerMotor.set(powerBR);
  }
  private boolean isOriented(double currentAngle, double targetAngle) {
    boolean oriented = false;

    return oriented;
  }

  public void driveMotors(double power, double setAngle) {
    m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,power); // Uncomment to spin drive motors
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,power);
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,power);
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,power);
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
    //m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
    //m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
    //m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
    //m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);

    
    m_frontleftSteerMotor.set(0.0);
    m_frontrightSteerMotor.set(0.0);
    m_backleftSteerMotor.set(0.0);
    m_backrightSteerMotor.set(0.0);
  }

  public static void sans() {
    if(!orcs.isPlaying()) {
      orcs.clearInstruments();
      orcs.addInstrument(m_frontleftDriveMotor);
      orcs.addInstrument(m_frontrightDriveMotor);
      orcs.addInstrument(m_backleftDriveMotor);
      orcs.addInstrument(m_backrightDriveMotor);
      orcs.loadMusic("mega_imp.chrp");
      orcs.play();
    }
  }
  public static void stop_tone() {
    if(orcs.isPlaying()) {
      orcs.stop();
    }
  }

}

