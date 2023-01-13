// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ClimbSub extends SubsystemBase {
  Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
  Solenoid exampleSolenoidPH = new Solenoid(PneumaticsModuleType.REVPH, 4);
  TalonFX m_climbArmMotor = new TalonFX(15);

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    m_unfoldArms(0);
    m_getArmStatus(0);
    m_foldArms(true);
    m_toggleArmSeparation(0);
   m_setClimberArmPower(0.5);
   m_zeroClimberEncoders(0); 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  

  }

  private double m_getClimberEncoder() {
     
    return  m_climbArmMotor.getSelectedSensorPosition();
  }

  public void m_setClimberArmPower(double d) {
    m_climbArmMotor.set(ControlMode.PercentOutput, d);
  }
  private void m_zeroClimberEncoders(int i) {
  }
  private void getClimberEncoder() {
  }
  private void m_toggleArmSeparation(int i) {
  }
  private void m_foldArms(boolean b) {setInverted(true);
    }
  private void setInverted(boolean b) {
  }
  private void setDefaultCommand(boolean b) {
  }
  private void m_getArmStatus(int i) {
  }
  private void m_unfoldArms(int i) {
  }  
}