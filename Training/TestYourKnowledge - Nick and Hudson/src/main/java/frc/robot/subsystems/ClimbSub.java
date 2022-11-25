// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class ClimbSub extends SubsystemBase {
  /** Creates a new ClimbSub. */
  public ClimbSub() {
    Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
Solenoid exampleSolenoidPH = new Solenoid(PneumaticsModuleType.REVPH, 4);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
