// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DrivetrainSub;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final static int m_kDriverControllerPort = 0;
  private final static int m_kOperatorControllerPort = 1;

  XboxController m_driverController = new XboxController(m_kDriverControllerPort);
  XboxController m_operaterController = new XboxController(m_kOperatorControllerPort);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
  
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Buttons
    // ID F310                              XboxController
    // -- --------------------------------  --------------
    // 1  X (Blue)                          kA
    // 2  A (Green)                         kB
    // 3  B (Red)                           kX
    // 4  Y (Yellow)                        kY
    // 5  LB (Left-Bumper: top button)      kLeftBumper
    // 6  RB (Right-Bumper: top button)     kRightBumper
    // 7  LT (Left-Trigger: bottom button)  kBack
    // 8  RT (Right-Trigger: bottom button) kStart
    // 9  Select/Back (Above left joystick) kLeftStick
    // 10 Start (Above right joystick)      kRightStick
    // 11 L3 (Press left joystick)          ---
    // 12 R3 (Press right joystick)         ---
    
    // Axes
    // ID F310                    XboxController
    // -- ----------------------  ----------------
    // 0  Left Stick Horizontal   kLeftX
    // 1  Left Stick Vertical     kLeftY
    // 2  Right Stick Horizontal  KLeftTrigger
    // 3  Right Stick Vertical    KRightTrigger
    // 4  ---                     kRightX
    // 5  ---                     kRightY
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
