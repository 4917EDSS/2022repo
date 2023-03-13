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

  private static double fl_dir = 1.0;
  private static double fr_dir = -1.0;
  private static double bl_dir = 1.0;
  private static double br_dir = -1.0;

  private static double fl_encoderOffset = 55.54;
  private static double fr_encoderOffset = 161.54;
  private static double bl_encoderOffset = 16.35;
  private static double br_encoderOffset = 146.78;

  //private final CANCoderConfiguration m_CANConfig; // Configuration settings for encoders

  private final CANCoder m_encoderFrontLeft = new CANCoder(Constants.CanIds.kEncoderFL);

  private final CANCoder m_encoderFrontRight = new CANCoder(Constants.CanIds.kEncoderFR);

  private final CANCoder m_encoderBackLeft = new CANCoder(Constants.CanIds.kEncoderBL);

  private final CANCoder m_encoderBackRight = new CANCoder(Constants.CanIds.kEncoderBR);


  // Motors

  private final CANSparkMax m_frontleftSteerMotor =
      new CANSparkMax(Constants.CanIds.kSteeringMotorFL, CANSparkMax.MotorType.kBrushless);
  private static final TalonFX m_frontleftDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorFL);

  private final CANSparkMax m_frontrightSteerMotor =
      new CANSparkMax(Constants.CanIds.kSteeringMotorFR, CANSparkMax.MotorType.kBrushless);
  private static final TalonFX m_frontrightDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorFR);

  private final CANSparkMax m_backleftSteerMotor =
      new CANSparkMax(Constants.CanIds.kSteeringMotorBL, CANSparkMax.MotorType.kBrushless);
  private static final TalonFX m_backleftDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorBL);

  private final CANSparkMax m_backrightSteerMotor =
      new CANSparkMax(Constants.CanIds.kSteeringMotorBR, CANSparkMax.MotorType.kBrushless);
  private static final TalonFX m_backrightDriveMotor = new TalonFX(Constants.CanIds.kDriveMotorBR);


  private static final AHRS m_gyro = new AHRS(Port.kMXP);


  private final PIDController pid = new PIDController(0.1, 0, 0.0);

  public SwerveDrivetrain() {
    resetGyro();
    SmartDashboard.putNumber("FL Offset Set", fl_encoderOffset);
    SmartDashboard.putNumber("FR Offset Set", fr_encoderOffset);
    SmartDashboard.putNumber("BL Offset Set", bl_encoderOffset);
    SmartDashboard.putNumber("BR Offset Set", br_encoderOffset);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Display Encoder angles and velocities
    SmartDashboard.putNumber("Front Left Angle (ABS): ", m_encoderFrontLeft.getAbsolutePosition() - fl_encoderOffset);
    SmartDashboard.putNumber("Front Right Angle (ABS): ", m_encoderFrontRight.getAbsolutePosition() - fr_encoderOffset);
    SmartDashboard.putNumber("Back Left Angle (ABS): ", m_encoderBackLeft.getAbsolutePosition() - bl_encoderOffset);
    SmartDashboard.putNumber("Back Right Angle (ABS): ", m_encoderBackRight.getAbsolutePosition() - br_encoderOffset);

    SmartDashboard.putNumber("Front Left ID: ", m_encoderFrontLeft.getDeviceID());
    SmartDashboard.putNumber("Front Right ID: ", m_encoderFrontRight.getDeviceID());
    SmartDashboard.putNumber("Back Left ID: ", m_encoderBackLeft.getDeviceID());
    SmartDashboard.putNumber("Back Right ID: ", m_encoderBackRight.getDeviceID());

    SmartDashboard.putNumber("Robot Gyro: ", getHeading());

    kP = SmartDashboard.getNumber("kP", 0.01);
    kI = SmartDashboard.getNumber("kI", 0.0);
    kD = SmartDashboard.getNumber("kD", 0.0);

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

    //fl_encoderOffset = SmartDashboard.getNumber("FL Offset Set", fl_encoderOffset);
    //fr_encoderOffset = SmartDashboard.getNumber("FR Offset Set", fr_encoderOffset);
    //bl_encoderOffset = SmartDashboard.getNumber("BL Offset Set", bl_encoderOffset);
    //br_encoderOffset = SmartDashboard.getNumber("BR Offset Set", br_encoderOffset);

    SmartDashboard.putNumber("FL Offset", fl_encoderOffset);
    SmartDashboard.putNumber("FR Offset", fr_encoderOffset);
    SmartDashboard.putNumber("BL Offset", bl_encoderOffset);
    SmartDashboard.putNumber("BR Offset", br_encoderOffset);

    /*
     * SmartDashboard.putBoolean("FL In front", inFront(0));
     * SmartDashboard.putBoolean("FR In front", inFront(1));
     * SmartDashboard.putBoolean("BL In front", inFront(2));
     * SmartDashboard.putBoolean("BR In front", inFront(3));
     */

    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  public void tuneOffsets() {
    fl_encoderOffset = m_encoderFrontLeft.getAbsolutePosition();
    fr_encoderOffset = m_encoderFrontRight.getAbsolutePosition();
    bl_encoderOffset = m_encoderBackLeft.getAbsolutePosition();
    br_encoderOffset = m_encoderBackRight.getAbsolutePosition();

  }
  // Steering functions

  public double vecToAngle(double x, double y) { // Convert 2d vectors to angles in degrees
    return Math.atan2(x, y) * 180.0f / 3.14159;
  }

  public double angleDiff(double a, double b) { // find difference between angles (can be positive or negative)
    return (a - b + 540.0) % 360 - 180;
  }

  private double clamp(double a, double mini, double maxi) {
    return Math.min(Math.max(a, mini), maxi);
  }

  public static void resetGyro() {
    m_gyro.reset();
  }

  private static double modulo(double x, double b) {
    return x - Math.floor(x / b) * b;
  }

  public static double getHeading() {
    double angle = modulo(m_gyro.getAngle(), 360.0);
    if(angle >= 0) {
      return angle;
    } else {
      return (360.0 - angle);
    }
  }


  public double getGlobalSteeringAngle(int motor) {
    double angl = 0.0;
    if(motor == 0) {
      angl = m_encoderFrontLeft.getAbsolutePosition() - fl_encoderOffset - getHeading();
    } else if(motor == 1) {
      angl = m_encoderFrontRight.getAbsolutePosition() - fr_encoderOffset - getHeading();
    } else if(motor == 2) {
      angl = m_encoderBackLeft.getAbsolutePosition() - bl_encoderOffset - getHeading();
    } else if(motor == 3) {
      angl = m_encoderBackRight.getAbsolutePosition() - br_encoderOffset - getHeading();
    }
    return modulo(angl, 360.0);
  }

  public boolean inFront(int motor) { // Is the motor at the front of the robot
    boolean inFron = false;
    switch(motor) {
      case 0:
        if((getHeading() - 180) >= -45.0 && (getHeading() - 180) < 135.0) {
          inFron = true;
        } else {
          inFron = false;
        }
        break;

      case 1:
        if((getHeading() - 180) >= -135.0 && (getHeading() - 180) < 45.0) {
          inFron = true;
        } else {
          inFron = false;
        }
        break;
      case 2:
        if((getHeading() - 180) >= -135.0 && (getHeading() - 180) < 45.0) {
          inFron = false;
        } else {
          inFron = true;
        }
        break;
      case 3:
        if((getHeading() - 180) >= -45.0 && (getHeading() - 180) < 135.0) {
          inFron = false;
        } else {
          inFron = true;
        }
        break;
    }
    return inFron;
  }

  public boolean inFront(int motor, double offset) { // Is the motor at the front of the robot
    boolean inFron = false;
    double gyroAngle = modulo(getHeading() - offset, 360.0) - 180.0;
    switch(motor) {
      case 0:
        if(gyroAngle > -45.0 && gyroAngle < 135.0) {
          inFron = true;
        } else {
          inFron = false;
        }
        SmartDashboard.putBoolean("FL In front", inFron);
        break;

      case 1:
        if(gyroAngle > -135.0 && gyroAngle < 45.0) {
          inFron = true;
        } else {
          inFron = false;
        }
        SmartDashboard.putBoolean("FR In front", inFron);
        break;
      case 2:
        if(gyroAngle > -135.0 && gyroAngle < 45.0) {
          inFron = false;
        } else {
          inFron = true;
        }
        SmartDashboard.putBoolean("BL In front", inFron);
        break;
      case 3:
        if(gyroAngle > -45.0 && gyroAngle < 135.0) {
          inFron = false;
        } else {
          inFron = true;
        }
        SmartDashboard.putBoolean("BR In front", inFron);
        break;
    }
    return inFron;
  }

  public boolean inFrontBad(int motor) { // Is the motor at the front of the robot
    boolean inFront = false;
    switch(motor) {
      case 0:
        if(angleDiff(getHeading(), 135.0) >= 90) {
          inFront = true;
        } else {
          inFront = false;
        }
        break;

      case 1:
        if(angleDiff(getHeading(), 225.0) > 90) {
          inFront = true;
        } else {
          inFront = false;
        }
        break;
      case 2:
        if(angleDiff(getHeading(), 45.0) > 90) {
          inFront = true;
        } else {
          inFront = false;
        }
        break;
      case 3:
        if(angleDiff(getHeading(), 315.0) >= 90) {
          inFront = true;
        } else {
          inFront = false;
        }
        break;
    }
    return inFront;
  }

  private void setAngle(int motor, double angle) { // 0 - FL, 1 - FR, 2 - BL, 3 - BR
    pid.enableContinuousInput(0, 180);
    if(motor == 0) {
      double powerFL =
          MathUtil.clamp(
              pid.calculate(m_encoderFrontLeft.getAbsolutePosition() - fl_encoderOffset, modulo(angle, 180.0)),
              -m_steeringPower, m_steeringPower);
      m_frontleftSteerMotor.set(m_steeringPower * powerFL);
    } else if(motor == 1) {
      double powerFR =
          MathUtil.clamp(
              pid.calculate(m_encoderFrontRight.getAbsolutePosition() - fr_encoderOffset, modulo(angle, 180.0)),
              -m_steeringPower, m_steeringPower);
      m_frontrightSteerMotor.set(m_steeringPower * powerFR);
    } else if(motor == 2) {
      double powerBL =
          MathUtil.clamp(
              pid.calculate(m_encoderBackLeft.getAbsolutePosition() - bl_encoderOffset, modulo(angle, 180.0)),
              -m_steeringPower, m_steeringPower);
      m_backleftSteerMotor.set(m_steeringPower * powerBL);
    } else if(motor == 3) {
      double powerBR =
          MathUtil.clamp(
              pid.calculate(m_encoderBackRight.getAbsolutePosition() - br_encoderOffset, modulo(angle, 180.0)),
              -m_steeringPower, m_steeringPower);
      m_backrightSteerMotor.set(m_steeringPower * powerBR);
    }
  }

  private void setGlobalAngle(int motor, double angle) { // 0 - FL, 1 - FR, 2 - BL, 3 - BR
    pid.enableContinuousInput(0, 180);
    double motor_angle = getGlobalSteeringAngle(motor);
    if(motor == 0) {
      double powerFL =
          MathUtil.clamp(pid.calculate(motor_angle, modulo(angle, 180.0)), -m_steeringPower, m_steeringPower);
      m_frontleftSteerMotor.set(m_steeringPower * powerFL);
    } else if(motor == 1) {
      double powerFR =
          MathUtil.clamp(pid.calculate(motor_angle, modulo(angle, 180.0)), -m_steeringPower, m_steeringPower);
      m_frontrightSteerMotor.set(m_steeringPower * powerFR);
    } else if(motor == 2) {
      double powerBL =
          MathUtil.clamp(pid.calculate(motor_angle, modulo(angle, 180.0)), -m_steeringPower, m_steeringPower);
      m_backleftSteerMotor.set(m_steeringPower * powerBL);
    } else if(motor == 3) {
      double powerBR =
          MathUtil.clamp(pid.calculate(motor_angle, modulo(angle, 180.0)), -m_steeringPower, m_steeringPower);
      m_backrightSteerMotor.set(m_steeringPower * powerBR);
    }
  }


  public void setSteeringAngle(double angle) {
    setAngle(0, angle); // Front Left
    setAngle(1, angle); // Front Right
    setAngle(2, angle); // Back Left
    setAngle(3, angle); // Back Right
  }

  public void setGlobalSteeringAngle(double angle) {
    setGlobalAngle(0, angle);
    setGlobalAngle(1, angle);
    setGlobalAngle(2, angle);
    setGlobalAngle(3, angle);
  }

  private boolean isOriented(int motor, double targetAngle, double range) { // FL, FR, BL, BR
    boolean oriented = false;

    if(motor == 0) {
      if(Math.abs(
          angleDiff(m_encoderFrontLeft.getAbsolutePosition() - fl_encoderOffset + 180.0, targetAngle + 180.0)) < range)
        oriented = true;
    } else if(motor == 1) {
      if(Math.abs(
          angleDiff(m_encoderFrontRight.getAbsolutePosition() - fr_encoderOffset + 180.0, targetAngle + 180.0)) < range) // -180 - 180 to 0 - 360 range
        oriented = true;
    } else if(motor == 2) {
      if(Math.abs(
          angleDiff(m_encoderBackLeft.getAbsolutePosition() - bl_encoderOffset + 180.0, targetAngle + 180.0)) < range)
        oriented = true;
    } else if(motor == 3) {
      if(Math.abs(
          angleDiff(m_encoderBackRight.getAbsolutePosition() - br_encoderOffset + 180.0, targetAngle + 180.0)) < range)
        oriented = true;
    }
    return oriented;
  }

  private boolean isGlobalOriented(int motor, double targetAngle, double range) { // FL, FR, BL, BR
    boolean oriented = false;
    double motor_angle = getGlobalSteeringAngle(motor);
    if(Math.abs(angleDiff(motor_angle + 180.0, targetAngle + 180.0)) < range)
      oriented = true;
    return oriented;
  }

  public void driveMotors(double power, double tarAngle, double range, boolean global) {
    if(global) {
      m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,
          m_steeringPower * power * (isGlobalOriented(0, tarAngle, 30.0) ? fl_dir : -fl_dir));
      m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,
          m_steeringPower * power * (isGlobalOriented(1, tarAngle, 30.0) ? fr_dir : -fr_dir)); // Inverted
      m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,
          m_steeringPower * power * (isGlobalOriented(2, tarAngle, 30.0) ? bl_dir : -bl_dir));
      m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,
          m_steeringPower * power * (isGlobalOriented(3, tarAngle, 10.0) ? br_dir : -br_dir));
    } else {
      m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,
          m_steeringPower * power * (isOriented(0, tarAngle, 30.0) ? fl_dir : -fl_dir));
      m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,
          m_steeringPower * power * (isOriented(1, tarAngle, 30.0) ? fr_dir : -fr_dir)); // Inverted
      m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,
          m_steeringPower * power * (isOriented(2, tarAngle, 30.0) ? bl_dir : -bl_dir));
      m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,
          m_steeringPower * power * (isOriented(3, tarAngle, 10.0) ? br_dir : -br_dir));
    }
  }

  public void driveMotor(int motor, double power, double tarAngle, double range, boolean global) {
    switch(motor) {
      case 0:
        if(global) {
          m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,
              m_steeringPower * power * (isGlobalOriented(0, tarAngle, range) ? fl_dir : -fl_dir));
        } else {
          m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,
              m_steeringPower * power * (isOriented(0, tarAngle, range) ? fl_dir : -fl_dir));
        }
        break;

      case 1:
        if(global) {
          m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,
              m_steeringPower * power * (isGlobalOriented(1, tarAngle, range) ? fr_dir : -fr_dir));
        } else {
          m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,
              m_steeringPower * power * (isOriented(1, tarAngle, range) ? fr_dir : -fr_dir));
        }
        break;

      case 2:
        if(global) {
          m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,
              m_steeringPower * power * (isGlobalOriented(2, tarAngle, range) ? bl_dir : -bl_dir));
        } else {
          m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,
              m_steeringPower * power * (isOriented(2, tarAngle, range) ? bl_dir : -bl_dir));
        }
        break;

      case 3:
        if(global) {
          m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,
              m_steeringPower * power * (isGlobalOriented(3, tarAngle, range) ? br_dir : -br_dir));
        } else {
          m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,
              m_steeringPower * power * (isOriented(3, tarAngle, range) ? br_dir : -br_dir));
        }
        break;
    }
  }

  public void circularDrive(double power) {

    setAngle(0, 135); // Turn steering motors in circular direction
    setAngle(1, 45);
    setAngle(2, 225);
    setAngle(3, 315);

    m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * power * (isOriented(0, 135, 30.0) ? fl_dir : -fl_dir));
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * power * (isOriented(1, 45, 30.0) ? fr_dir : -fr_dir)); // Inverted
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * power * (isOriented(2, 225, 30.0) ? bl_dir : -bl_dir));
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * power * (isOriented(3, 315, 30.0) ? br_dir : -br_dir));
  }

  public void brakes() { // There is no drive
    setAngle(2, 135); // Turn steering motors in circular direction
    setAngle(3, 45);
    setAngle(0, 225);
    setAngle(1, 315);
  }

  public void arcadeSwerve(double fwdPower, double turnPower, double dir) {
    double turnDirFL = turnPower * 45.0 * (inFront(0, dir) ? 1.0 : -1.0);
    double turnDirFR = turnPower * 45.0 * (inFront(1, dir) ? 1.0 : -1.0);
    double turnDirBL = turnPower * 45.0 * (inFront(2, dir) ? 1.0 : -1.0);
    double turnDirBR = turnPower * 45.0 * (inFront(3, dir) ? 1.0 : -1.0);
    setGlobalAngle(0, dir + turnDirFL);
    setGlobalAngle(1, dir + turnDirFR);
    setGlobalAngle(2, dir + turnDirBL);
    setGlobalAngle(3, dir + turnDirBR);

    driveMotor(0, fwdPower, dir + turnDirFL, 30.0, true);
    driveMotor(1, fwdPower, dir + turnDirFR, 30.0, true);
    driveMotor(2, fwdPower, dir + turnDirBL, 30.0, true);
    driveMotor(3, fwdPower, dir + turnDirBR, 30.0, true);
    /*
     * m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,
     * m_steeringPower * fwdPower * fl_dir);
     * m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,
     * m_steeringPower * fwdPower * fr_dir);
     * m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,
     * m_steeringPower * fwdPower * bl_dir);
     * m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,
     * m_steeringPower * fwdPower * br_dir);
     */
  }

  public void arcadeSwerveDemo(double fwdPower, double turnPower) {
    double turnDirFL = turnPower * 45.0 * 1;
    double turnDirFR = turnPower * 45.0 * 1;
    double turnDirBL = turnPower * 45.0 * -1;
    double turnDirBR = turnPower * 45.0 * -1;

    setAngle(0, turnDirFL);
    setAngle(1, turnDirFR);
    setAngle(2, turnDirBL);
    setAngle(3, turnDirBR);

    m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * fwdPower * fl_dir);
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * fwdPower * fr_dir);
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * fwdPower * -bl_dir);
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * fwdPower * -br_dir);
  }

  public void circularStrafe(double power, double angle) { // None of this code works
    double vxFL = clamp(Math.sin(135 * 3.1415 / 180), -1.0, 1.0);
    double vxFR = clamp(Math.sin(45 * 3.1415 / 180), -1.0, 1.0);
    double vxBL = clamp(Math.sin(225 * 3.1415 / 180), -1.0, 1.0);
    double vxBR = clamp(Math.sin(315 * 3.1415 / 180), -1.0, 1.0);

    double vyFL = clamp(Math.cos(135 * 3.1415 / 180), -1.0, 1.0);
    double vyFR = clamp(Math.cos(45 * 3.1415 / 180), -1.0, 1.0);
    double vyBL = clamp(Math.cos(225 * 3.1415 / 180), -1.0, 1.0);
    double vyBR = clamp(Math.cos(315 * 3.1415 / 180), -1.0, 1.0);

    double gxFL = clamp(Math.sin((getGlobalSteeringAngle(0) + angle) * 3.1415 / 180), -1.0, 1.0);
    double gxFR = clamp(Math.sin((getGlobalSteeringAngle(1) + angle) * 3.1415 / 180), -1.0, 1.0);
    double gxBL = clamp(Math.sin((getGlobalSteeringAngle(2) + angle) * 3.1415 / 180), -1.0, 1.0);
    double gxBR = clamp(Math.sin((getGlobalSteeringAngle(3) + angle) * 3.1415 / 180), -1.0, 1.0);

    double gyFL = clamp(Math.cos((getGlobalSteeringAngle(0) + angle) * 3.1415 / 180), -1.0, 1.0);
    double gyFR = clamp(Math.cos((getGlobalSteeringAngle(1) + angle) * 3.1415 / 180), -1.0, 1.0);
    double gyBL = clamp(Math.cos((getGlobalSteeringAngle(2) + angle) * 3.1415 / 180), -1.0, 1.0);
    double gyBR = clamp(Math.cos((getGlobalSteeringAngle(3) + angle) * 3.1415 / 180), -1.0, 1.0);

    double jx = clamp(Math.sin((angle + 180) * 3.1415 / 180), -1.0, 1.0); //Joystick
    double jy = clamp(Math.sin((angle + 180) * 3.1415 / 180), -1.0, 1.0);


    double frontleft = vecToAngle(vxFL + jx, vyFL + jy);
    double frontright = vecToAngle(vxFR + jx, vyFR + jy);
    double backleft = vecToAngle(vxBL + jx, vyBL + jy);
    double backright = vecToAngle(vxBR + jx, vyBR + jy);

    setGlobalAngle(0, frontleft);
    setGlobalAngle(1, frontright);
    setGlobalAngle(2, backleft);
    setGlobalAngle(3, backright);

    m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * power * (isGlobalOriented(0, frontleft, 30.0) ? 1.0 : -1.0));
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * power * (isGlobalOriented(1, frontright, 30.0) ? 1.0 : -1.0)); // Inverted
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * power * (isGlobalOriented(2, backleft, 30.0) ? 1.0 : -1.0));
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput,
        m_steeringPower * power * (isGlobalOriented(3, backright, 30.0) ? -1.0 : 1.0));
  }

  public void brakeDrive() { // Set all drive motors to 0 power
    m_frontleftDriveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    m_frontrightDriveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    m_backleftDriveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    m_backrightDriveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
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

