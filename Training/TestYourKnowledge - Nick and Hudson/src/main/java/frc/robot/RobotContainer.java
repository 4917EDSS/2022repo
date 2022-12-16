// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DrivetrainSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveBackwardCmd;
import frc.robot.subsystems.ClimbSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final static int m_kDriverControllerPort = 0;
  private final static int m_kOperatorControllerPort = 1;
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_drivetrainSub);

  private final PS4Controller m_driverController = new PS4Controller(m_kDriverControllerPort);
  private final PS4Controller m_operatorController = new
      PS4Controller(m_kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrainSub.setDefaultCommand(
      new DriveWithJoystickCmd(m_driverController, m_drivetrainSub));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, PS4Controller.Button.kCross.value)
      .whenHeld(new DriveForwardCmd(m_drivetrainSub));
      new JoystickButton(m_driverController, PS4Controller.Button.kSquare.value)
        .whenHeld(new DriveBackwardCmd(m_drivetrainSub));
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
