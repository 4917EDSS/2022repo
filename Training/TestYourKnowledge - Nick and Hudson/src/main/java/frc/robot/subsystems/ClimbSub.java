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
public class ClimbSub extends SubsystemBase {
  /** Creates a new ClimbSub. */
  public ClimbSub() {
    Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
Solenoid exampleSolenoidPH = new Solenoid(PneumaticsModuleType.REVPH, 4);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   m_unfoldArms(0);
   m_getArmStatus(0);
   m_foldArms(true);
   m_toggleArmSeparation(0);
  m_setClimberArmPower(0.5);
  m_zeroClimberEncoders(0); 
  m_getClimberEncoder(0);

  }
  private void m_getClimberEncoder(int i) {
  }
  private void m_setClimberArmPower(double d) {
  }
  private void m_zeroClimberEncoders(int i) {
  }
  private void getClimberEncoder() {
  }
  private void m_toggleArmSeparation(int i) {
  }
  private void m_foldArms(boolean b) {("Value", m_table_listener, true);
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