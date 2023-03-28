// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/*
 * For basic implementation
 * 
 * Use trueDrive(fwdPower, turnPower) to drive field oriented
 * 
 * 
 */

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
import frc.robot.Constants;

import frc.robot.Vector;


public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new SwerveDrivetrain. */

  private static final Orchestra orcs = new Orchestra();
  // Steering Encoders

  private double kP = 1.0;
  private double kI = 1.0;
  private double kD = 1.0;
  private double m_steeringPower = 0.5;

  private double fl_dir = 1.0;
  private double fr_dir = 1.0;
  private double bl_dir = 1.0;
  private double br_dir = -1.0;

  private double fl_encoderOffset = 49.21;//55.54;
  private double fr_encoderOffset = 313.5;//151.54;
  private double bl_encoderOffset = 5.27;//16.35;
  private double br_encoderOffset = 164.35;//146.78;

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

    /*
     * kP = SmartDashboard.getNumber("kP", 0.01);
     * kI = SmartDashboard.getNumber("kI", 0.0);
     * kD = SmartDashboard.getNumber("kD", 0.0);
     */

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

    fl_encoderOffset = SmartDashboard.getNumber("[Set] FL Offset", fl_encoderOffset);
    fr_encoderOffset = SmartDashboard.getNumber("[Set] FR Offset", fr_encoderOffset);
    bl_encoderOffset = SmartDashboard.getNumber("[Set] BL Offset", bl_encoderOffset);
    br_encoderOffset = SmartDashboard.getNumber("[Set] BR Offset", br_encoderOffset);

    SmartDashboard.putNumber("FL Offset", fl_encoderOffset);
    SmartDashboard.putNumber("FR Offset", fr_encoderOffset);
    SmartDashboard.putNumber("BL Offset", bl_encoderOffset);
    SmartDashboard.putNumber("BR Offset", br_encoderOffset);

    fl_dir = (SmartDashboard.getBoolean("Invert FL", (fl_dir == -1.0))) ? -1.0 : 1.0;
    fr_dir = (SmartDashboard.getBoolean("Invert FR", (fr_dir == -1.0))) ? -1.0 : 1.0;
    bl_dir = (SmartDashboard.getBoolean("Invert BL", (bl_dir == -1.0))) ? -1.0 : 1.0;
    br_dir = (SmartDashboard.getBoolean("Invert BR", (br_dir == -1.0))) ? -1.0 : 1.0;

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
  // *** Conversion functions ***

  public double vecToAngle(double x, double y) { // Convert 2d vectors to angles in degrees
    return Math.atan2(y, x) * 180.0f / 3.14159;
  }

  public Vector angleToVec(double dir, double power) { // Angle in 360 degrees, power between -1 and 1
    double rad = dir * 3.14159 / 180.0;

    double x = Math.cos(rad);
    double y = Math.sin(rad);

    Vector val = new Vector(x, y);
    val.mul(power);
    return val;
  }

  public double angleDiff(double a, double b) { // find difference between angles (can be positive or negative)
    return (a - b + 540.0) % 360 - 180;
  }

  private double clamp(double a, double mini, double maxi) {
    return Math.min(Math.max(a, mini), maxi);
  }

  public double getPower(Vector a) {
    return MathUtil.clamp(a.length(), -1.0, 1.0);
  }

  public double getDirection(Vector a) {
    return vecToAngle(a.x, a.y);
  }

  public static void resetGyro() {
    m_gyro.reset();
  }

  private static double modulo(double x, double b) {
    return x - Math.floor(x / b) * b;
  }

  // *** Get functions ***

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

  // *** Set functions ***

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

  // *** Drive functions ***

  public void driveVector(Vector Vfl, Vector Vfr, Vector Vbl, Vector Vbr, boolean global) { // The good one

    Vector fl = Vector.normalize(Vfl);
    Vector fr = Vector.normalize(Vfr);
    Vector bl = Vector.normalize(Vbl);
    Vector br = Vector.normalize(Vbr);

    double flAng = getDirection(Vfl);
    double frAng = getDirection(Vfr);
    double blAng = getDirection(Vbl);
    double brAng = getDirection(Vbr);

    double flPow = getPower(Vfl);
    double frPow = getPower(Vfr);
    double blPow = getPower(Vbl);
    double brPow = getPower(Vbr);
    if(!global) {
      setAngle(0, flAng);
      setAngle(1, frAng);
      setAngle(2, blAng);
      setAngle(3, brAng);
    } else {
      setGlobalAngle(0, flAng);
      setGlobalAngle(1, frAng);
      setGlobalAngle(2, blAng);
      setGlobalAngle(3, brAng);
    }


    driveMotor(0, flPow, flAng, 30.0, global);
    driveMotor(1, frPow, frAng, 30.0, global);
    driveMotor(2, blPow, blAng, 30.0, global);
    driveMotor(3, brPow, brAng, 30.0, global);
  }

  public void trueDrive(Vector fwd, double turnPower) {
    Vector flTurn = angleToVec(modulo(315.0 - getHeading(), 360), 1.0);
    Vector frTurn = angleToVec(modulo(225.0 - getHeading(), 360), 1.0);
    Vector blTurn = angleToVec(modulo(45.0 - getHeading(), 360), 1.0);
    Vector brTurn = angleToVec(modulo(135.0 - getHeading(), 360), 1.0);

    flTurn.mul(turnPower);
    frTurn.mul(turnPower);
    blTurn.mul(turnPower);
    brTurn.mul(turnPower);

    Vector fl = fwd.clone();
    Vector fr = fwd.clone();
    Vector bl = fwd.clone();
    Vector br = fwd.clone();

    fl.add(flTurn);
    fr.add(frTurn);
    bl.add(blTurn);
    br.add(brTurn);

    driveVector(fl, fr, bl, br, true);
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

