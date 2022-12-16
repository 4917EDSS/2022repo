// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;


public class SwerveWithJoystickCmd extends CommandBase {
  /** Creates a new SwerveWithJoystickCmd. */

  SwerveDrivetrain m_swervetrainSub;
  PS4Controller m_controller;


  public SwerveWithJoystickCmd(SwerveDrivetrain swerveSub,PS4Controller controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swervetrainSub = swerveSub;
    m_controller = controller;
    addRequirements(swerveSub);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angl = 0.0;
    double power = 0.0f;

    double xVector = m_controller.getLeftX();
    double yVector = m_controller.getLeftY();

    if(m_controller.getSquareButton()) {
      if(m_controller.getRawAxis(Constants.R2Axis) > 0.1) { // Forwards
        power = m_controller.getRawAxis(Constants.R2Axis);
      }
      else if(m_controller.getRawAxis(Constants.L2Axis) > 0.1) { //Backwards
        power = -m_controller.getRawAxis(Constants.L2Axis);
      }
      else {
        power = 0.0;
      }
      m_swervetrainSub.circularDrive(power);
    }
    else if(m_controller.getCrossButton()) {
      if(xVector+yVector != 0) {
        angl = m_swervetrainSub.vecToAngle(xVector, yVector);
        m_swervetrainSub.setGlobalSteeringAngle(angl);
        SmartDashboard.putNumber("Angle Controller", angl);
      }
      else {
        m_swervetrainSub.brakeSteer(); // Comment this out for the funny
      }

      if(m_controller.getRawAxis(Constants.R2Axis) > 0.1) { // Forwards
        m_swervetrainSub.driveMotors(m_controller.getRawAxis(Constants.R2Axis), angl, 10.0, true);
      }
      else if(m_controller.getRawAxis(Constants.L2Axis) > 0.1) { //Backwards
        m_swervetrainSub.driveMotors(-m_controller.getRawAxis(Constants.L2Axis), angl, 10.0, true);
      }
      else {
        m_swervetrainSub.brakeDrive(); //Also comment for the funny
      }
      /*
      if(xVector+yVector != 0) {
        angl = m_swervetrainSub.vecToAngle(xVector, yVector);
        SmartDashboard.putNumber("Angle Controller", angl);
      }

      if(m_controller.getRawAxis(Constants.R2Axis) > 0.1) { // Forwards
        power = 0.3*m_controller.getRawAxis(Constants.R2Axis);
      }
      else if(m_controller.getRawAxis(Constants.L2Axis) > 0.1) { //Backwards
        power = -0.3*m_controller.getRawAxis(Constants.L2Axis);
      }
      else {
        power = 0.0; //Also comment for the funny
      }
      m_swervetrainSub.circularStrafe(power, angl);*/
    }
    else{
      if(xVector+yVector != 0) {
        angl = m_swervetrainSub.vecToAngle(xVector, yVector);
        m_swervetrainSub.setSteeringAngle(angl);
        SmartDashboard.putNumber("Angle Controller", angl);
      }
      else {
        m_swervetrainSub.brakeSteer(); // Comment this out for the funny
      }

      if(m_controller.getRawAxis(Constants.R2Axis) > 0.1) { // Forwards
        m_swervetrainSub.driveMotors(m_controller.getRawAxis(Constants.R2Axis), angl, 10.0, false);
      }
      else if(m_controller.getRawAxis(Constants.L2Axis) > 0.1) { //Backwards
        m_swervetrainSub.driveMotors(-m_controller.getRawAxis(Constants.L2Axis), angl, 10.0, false);
      }
      else {
        m_swervetrainSub.brakeDrive(); //Also comment for the funny
      }
    }
    
    /*
    if(m_controller.getTriangleButton()) {
      angl = 0.0f;
      m_swervetrainSub.setAngleFL(angl);
    }
    else if(m_controller.getCircleButton()) {
      angl = 90.0f;
      m_swervetrainSub.setAngleFL(angl);
    }
    else if(m_controller.getCrossButton()) {
      angl = 180.0f;
      m_swervetrainSub.setAngleFL(angl);
    }
    else if(m_controller.getSquareButton()) {
      angl = 270.0f;
      m_swervetrainSub.setAngleFL(angl);
    }*/
    
    

    /*

    double stSpeed0 = m_controller.getTriangleButton() ? 0.1 : 0.0;
    double stSpeed1 = m_controller.getCircleButton() ? 0.1 : 0.0;
    double stSpeed3 = m_controller.getCrossButton() ? 0.1 : 0.0;
    double stSpeed2 = m_controller.getSquareButton() ? 0.1 : 0.0;

    double drSpeed0 = m_controller.getL1Button() ? 0.1 : 0.0;
    double drSpeed1 = m_controller.getR1Button() ? 0.1 : 0.0;
    double drSpeed2 = m_controller.getL2Button() ? 0.1 : 0.0;
    double drSpeed3 = m_controller.getR2Button() ? 0.1 : 0.0;
    m_swervetrainSub.steerMotor(0, stSpeed0);
    m_swervetrainSub.steerMotor(1, stSpeed1);
    m_swervetrainSub.steerMotor(2, stSpeed2);
    m_swervetrainSub.steerMotor(3, stSpeed3);

    m_swervetrainSub.driveMotor(0, drSpeed0);
    m_swervetrainSub.driveMotor(1, drSpeed1);
    m_swervetrainSub.driveMotor(2, drSpeed2);
    m_swervetrainSub.driveMotor(3, drSpeed3);
    */

    if(m_controller.getOptionsButton()) {
      m_swervetrainSub.sans();
    }
    else if(m_controller.getShareButton()) {
      m_swervetrainSub.stop_tone();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
