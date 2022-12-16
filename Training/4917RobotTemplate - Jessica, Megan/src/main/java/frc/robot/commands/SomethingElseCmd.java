// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
/*-----------------------------------------------------------------------
| Import the classes that you need to use in this subsystem.            |
| Keep them in alphabetical order, including existing ones.             |
|                                                                       |
-----------------------------------------------------------------------*/

public class SomethingElseCmd extends CommandBase {
  /*-----------------------------------------------------------------------
  | Member variables                                                      |
  | -  Available to the entire class.                                     |
  | -  Start with m_                                                      |
  | Examples                                                              |
  | - "private" to hold constructor parameters (m_drivetrainSub)          |
  | - "private final" internal constants (motor power, distance to move)  |
  |                                                                       |
  -----------------------------------------------------------------------*/

  /** Creates a new SomethingElseCmd. */
  public SomethingElseCmd() {
    /*-----------------------------------------------------------------------
    | Import the classes that you need to use in this subsystem.            |
    | Keep them in alphabetical order, including existing ones.             |
    |                                                                       |
    -----------------------------------------------------------------------*/
    // Use addRequirements() here to declare subsystem dependencies.
    /*-----------------------------------------------------------------------
    | addRequirements(subsystem) calls                                      |
    | - One for each subsystem that this command affects                    |
    | - Don't include subsystems that you just read from (e.g. getDistance) |
    |                                                                       |
    -----------------------------------------------------------------------*/

    /*-----------------------------------------------------------------------
    | Any setup or reset that should be done when the robot powers up.      |
    |                                                                       |
    -----------------------------------------------------------------------*/
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*-----------------------------------------------------------------------
    | Any setup or reset that should be done when the command starts.       |
    |                                                                       |
    -----------------------------------------------------------------------*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*-----------------------------------------------------------------------
    | Any updates that need to be made while the command runs (50x per sec) |
    |                                                                       |
    -----------------------------------------------------------------------*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*-----------------------------------------------------------------------
    | Do any cleanup that you have to do when the command ends.             |
    | - If interrupted is true, you might need to do more cleanup           |
    |                                                                       |
    -----------------------------------------------------------------------*/
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*-----------------------------------------------------------------------
    | Check if the command has reached its end condition (50x per sec)      |
    | - Return true if it has                                               |
    |                                                                       |
    -----------------------------------------------------------------------*/
    return false;
  }
}
