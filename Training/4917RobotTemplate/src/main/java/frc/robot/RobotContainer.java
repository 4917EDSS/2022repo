// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
/*-----------------------------------------------------------------------
| Import the classes that you need to use in this subsystem.            |
| Keep them in alphabetical order, including existing ones.             |
|                                                                       |
-----------------------------------------------------------------------*/

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /*-----------------------------------------------------------------------
  | Member variables                                                      |
  | -  Available to the entire class.                                     |
  | -  Start with m_                                                      |
  |                                                                       |
  | Examples                                                              |
  | - "private final" controller IDs (e.g. m_kDriverControllerPort)       |
  | - "private final" controllers (e.g. m_driverController)               |
  | - "private final" subsystems (e.g. m_drivetrainSub)                   |
  |                                                                       |
  | Note:                                                                 |
  | - We don't usually define our auto command here.  We have multiple    |
  |   commands to choose from and use the SmartDashboard to select it.    |
  |                                                                       |
  -----------------------------------------------------------------------*/

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*-----------------------------------------------------------------------
    | Set default subsystem commands.                                       |
    |                                                                       |
    -----------------------------------------------------------------------*/

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*-----------------------------------------------------------------------
    | Create controller buttons and connect them to commands                |
    |                                                                       |
    -----------------------------------------------------------------------*/

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
