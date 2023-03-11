// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI.Port;
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
  private double m_steeringPower = 0.5;

  private static double fl_encoderOffset = 37.6;
  private static double fr_encoderOffset = 228.0;
  private static double bl_encoderOffset = 7.6;
  private static double br_encoderOffset = 110.5;
  
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


  private static final AHRS m_gyro = new AHRS(Port.kMXP);


  private final PIDController pid = new PIDController(0.1, 0, 0.0);
  public SwerveDrivetrain() {
    resetGyro();
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

    SmartDashboard.putNumber("Robot Gyro: ", getHeading()%360);

    kP = SmartDashboard.getNumber("kP", 0.01);
    kI = SmartDashboard.getNumber("kI", 0.0);
    kD = SmartDashboard.getNumber("kD", 0.0);

    SmartDashboard.putNumber("kP",kP);
    SmartDashboard.putNumber("kI",kI);
    SmartDashboard.putNumber("kD",kD);

    fl_encoderOffset = SmartDashboard.getNumber("FL Offset", fl_encoderOffset);
    fr_encoderOffset = SmartDashboard.getNumber("FR Offset", fr_encoderOffset);
    bl_encoderOffset = SmartDashboard.getNumber("BL Offset", bl_encoderOffset);
    br_encoderOffset = SmartDashboard.getNumber("BR Offset", br_encoderOffset);

    SmartDashboard.putNumber("FL Offset", fl_encoderOffset);
    SmartDashboard.putNumber("FR Offset", fr_encoderOffset);
    SmartDashboard.putNumber("BL Offset", bl_encoderOffset);
    SmartDashboard.putNumber("BR Offset", br_encoderOffset);
    
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  
  // Steering functions

  public double vecToAngle(double x, double y) { // Convert 2d vectors to angles in degrees
    return Math.atan2(x,y) * 180.0f / 3.14159;
  }

  public double angleDiff(double a, double b) { // find difference between angles (can be positive or negative)
    return (a - b + 540.0) % 360 - 180;
  } 

  private double clamp(double a, double mini, double maxi) {
    return Math.min(Math.max(a,mini),maxi);
  }

  public static void resetGyro(){
    m_gyro.reset(); 
  }
  
  public static double getHeading(){
    return m_gyro.getAngle(); 
  }

  public double getGlobalSteeringAngle(int motor) {
    double angl = 0.0;
    if(motor == 0) {
      angl = m_encoderFrontLeft.getAbsolutePosition() - fl_encoderOffset - getHeading();
    }
    else if(motor == 1) {
      angl = m_encoderFrontRight.getAbsolutePosition() - fr_encoderOffset - getHeading();
    }
    else if(motor == 2) {
      angl = m_encoderBackLeft.getAbsolutePosition() - bl_encoderOffset - getHeading();
    }
    else if(motor == 3) {
      angl = m_encoderBackRight.getAbsolutePosition() - br_encoderOffset - getHeading();
    }
    return angl%360;
  }

  private void setAngle(int motor, double angle) { // 0 - FL, 1 - FR, 2 - BL, 3 - BR
    pid.enableContinuousInput(0, 180);
    if (motor == 0) {
      double powerFL = MathUtil.clamp(pid.calculate(m_encoderFrontLeft.getAbsolutePosition()-fl_encoderOffset,angle%180),-m_steeringPower,m_steeringPower);
      m_frontleftSteerMotor.set(m_steeringPower*powerFL);
    }
    else if(motor == 1) {
      double powerFR = MathUtil.clamp(pid.calculate(m_encoderFrontRight.getAbsolutePosition()-fr_encoderOffset,angle%180),-m_steeringPower,m_steeringPower);
      m_frontrightSteerMotor.set(m_steeringPower*powerFR);
    }
    else if(motor == 2) {
      double powerBL = MathUtil.clamp(pid.calculate(m_encoderBackLeft.getAbsolutePosition()-bl_encoderOffset,angle%180),-m_steeringPower,m_steeringPower);
      m_backleftSteerMotor.set(m_steeringPower*powerBL);
    }
    else if(motor == 3) {
      double powerBR = MathUtil.clamp(pid.calculate(m_encoderBackRight.getAbsolutePosition()-br_encoderOffset,angle%180),-m_steeringPower,m_steeringPower);
      m_backrightSteerMotor.set(m_steeringPower*powerBR);
    }
  }
  private void setGlobalAngle(int motor, double angle) { // 0 - FL, 1 - FR, 2 - BL, 3 - BR
    pid.enableContinuousInput(0, 180);
    double motor_angle = getGlobalSteeringAngle(motor);
    if (motor == 0) {
      double powerFL = MathUtil.clamp(pid.calculate(motor_angle,angle%180),-m_steeringPower,m_steeringPower);
      m_frontleftSteerMotor.set(m_steeringPower*powerFL);
    }
    else if(motor == 1) {
      double powerFR = MathUtil.clamp(pid.calculate(motor_angle,angle%180),-m_steeringPower,m_steeringPower);
      m_frontrightSteerMotor.set(m_steeringPower*powerFR);
    }
    else if(motor == 2) {
      double powerBL = MathUtil.clamp(pid.calculate(motor_angle,angle%180),-m_steeringPower,m_steeringPower);
      m_backleftSteerMotor.set(m_steeringPower*powerBL);
    }
    else if(motor == 3) {
      double powerBR = MathUtil.clamp(pid.calculate(motor_angle,angle%180),-m_steeringPower,m_steeringPower);
      m_backrightSteerMotor.set(m_steeringPower*powerBR);
    }
  }
  public void setSteeringAngle(double angle) {
    setAngle(0,angle); // Front Left
    setAngle(1,angle); // Front Right
    setAngle(2,angle); // Back Left
    setAngle(3,angle); // Back Right
  }
  public void setGlobalSteeringAngle(double angle) {
    setGlobalAngle(0, angle);
    setGlobalAngle(1, angle);
    setGlobalAngle(2, angle);
    setGlobalAngle(3, angle);
  }
  private boolean isOriented(int motor, double targetAngle,double range) { // FL, FR, BL, BR
    boolean oriented = false;

    if (motor == 0) {
      if(Math.abs(angleDiff(m_encoderFrontLeft.getAbsolutePosition()-fl_encoderOffset+180.0,targetAngle+180.0)) < range)
      oriented = true;
    }
    else if(motor == 1) {
      if(Math.abs(angleDiff(m_encoderFrontRight.getAbsolutePosition()-fr_encoderOffset+180.0,targetAngle+180.0)) < range) // -180 - 180 to 0 - 360 range
      oriented = true;
    }
    else if(motor == 2) {
      if(Math.abs(angleDiff(m_encoderBackLeft.getAbsolutePosition()-bl_encoderOffset+180.0,targetAngle+180.0)) < range)
      oriented = true;
    }
    else if(motor == 3) {
      if(Math.abs(angleDiff(m_encoderBackRight.getAbsolutePosition()-br_encoderOffset+180.0,targetAngle+180.0)) < range)
      oriented = true;
    }
    return oriented;
  }
  private boolean isGlobalOriented(int motor, double targetAngle,double range) { // FL, FR, BL, BR
    boolean oriented = false;
    double motor_angle = getGlobalSteeringAngle(motor);
    if(Math.abs(angleDiff(motor_angle+180.0,targetAngle+180.0)) < range)
      oriented = true;
    return oriented;
  }

  public void driveMotors(double power, double tarAngle, double range, boolean global) {
    if(global) {
      m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isGlobalOriented(0, tarAngle, 30.0) ? 1.0 : -1.0));
      m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isGlobalOriented(1, tarAngle, 30.0) ? 1.0 : -1.0)); // Inverted
      m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isGlobalOriented(2, tarAngle, 30.0) ? 1.0 : -1.0));
      m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isGlobalOriented(3, tarAngle, 10.0) ? -1.0 : 1.0));
    }
    else {
      m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isOriented(0, tarAngle, 30.0) ? 1.0 : -1.0));
      m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isOriented(1, tarAngle, 30.0) ? 1.0 : -1.0)); // Inverted
      m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isOriented(2, tarAngle, 30.0) ? 1.0 : -1.0));
      m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isOriented(3, tarAngle, 10.0) ? -1.0 : 1.0));
    }
  }
  public void circularDrive(double power) {

    setAngle(0,135); // Turn steering motors in circular direction
    setAngle(1,45);
    setAngle(2,225);
    setAngle(3,315);

    m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isOriented(0, 135, 30.0) ? 1.0 : -1.0));
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isOriented(1, 45, 30.0) ? 1.0 : -1.0)); // Inverted
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isOriented(2, 225, 30.0) ? 1.0 : -1.0));
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isOriented(3, 315, 30.0) ? -1.0 : 1.0));
  }
  public void circularStrafe(double power, double angle) { // None of this code works
    double vxFL = clamp(Math.sin(135*3.1415/180),-1.0,1.0);
    double vxFR = clamp(Math.sin(45*3.1415/180 ),-1.0,1.0);
    double vxBL = clamp(Math.sin(225*3.1415/180),-1.0,1.0);
    double vxBR = clamp(Math.sin(315*3.1415/180),-1.0,1.0);

    double vyFL = clamp(Math.cos(135*3.1415/180),-1.0,1.0);
    double vyFR = clamp(Math.cos(45*3.1415/180 ),-1.0,1.0);
    double vyBL = clamp(Math.cos(225*3.1415/180),-1.0,1.0);
    double vyBR = clamp(Math.cos(315*3.1415/180),-1.0,1.0);

    double gxFL = clamp(Math.sin((getGlobalSteeringAngle(0)+angle)*3.1415/180),-1.0,1.0);
    double gxFR = clamp(Math.sin((getGlobalSteeringAngle(1)+angle)*3.1415/180),-1.0,1.0);
    double gxBL = clamp(Math.sin((getGlobalSteeringAngle(2)+angle)*3.1415/180),-1.0,1.0);
    double gxBR = clamp(Math.sin((getGlobalSteeringAngle(3)+angle)*3.1415/180),-1.0,1.0);

    double gyFL = clamp(Math.cos((getGlobalSteeringAngle(0)+angle)*3.1415/180),-1.0,1.0);
    double gyFR = clamp(Math.cos((getGlobalSteeringAngle(1)+angle)*3.1415/180),-1.0,1.0);
    double gyBL = clamp(Math.cos((getGlobalSteeringAngle(2)+angle)*3.1415/180),-1.0,1.0);
    double gyBR = clamp(Math.cos((getGlobalSteeringAngle(3)+angle)*3.1415/180),-1.0,1.0);

    double jx = clamp(Math.sin((angle+180)*3.1415/180),-1.0,1.0); //Joystick
    double jy = clamp(Math.sin((angle+180)*3.1415/180),-1.0,1.0);


    double frontleft = vecToAngle(vxFL + jx, vyFL + jy);
    double frontright = vecToAngle(vxFR + jx, vyFR + jy);
    double backleft = vecToAngle(vxBL + jx, vyBL + jy);
    double backright = vecToAngle(vxBR + jx, vyBR + jy);

    setGlobalAngle(0, frontleft);
    setGlobalAngle(1, frontright);
    setGlobalAngle(2, backleft);
    setGlobalAngle(3, backright);

    m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isGlobalOriented(0, frontleft, 30.0) ? 1.0 : -1.0));
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isGlobalOriented(1, frontright, 30.0) ? 1.0 : -1.0)); // Inverted
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isGlobalOriented(2, backleft, 30.0) ? 1.0 : -1.0));
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,m_steeringPower*power * (isGlobalOriented(3, backright, 30.0) ? -1.0 : 1.0));
  }
  public void brakeDrive() { // Set all drive motors to 0 power
    m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,0.0);
  }
  public void brakeSteer() { // Set all steering motors to 0 power
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

