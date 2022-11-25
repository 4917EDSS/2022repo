// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DrivetrainSub;

/*-----------------------------------------------------------------------
| Import the classes that you need to use in this subsystem.            |
| Keep them in alphabetical order, including existing ones.             |
|                                                                       |
-----------------------------------------------------------------------*/

public class SomethingSub extends SubsystemBase {
  
  /*-----------------------------------------------------------------------
  | Member variables                                                      |
  | -  Available to the entire class.                                     |
  | -  Start with m_                                                      |
  | Examples                                                              |
  | - "private final" hardware objects (motors, sensors, solenoids...)    |
  | - "private final" internal constants (default power for a motor)      |
  | - "public final" externally-accessible constants (enums, IDs)         |
  | - "private" state variables (last position)                           |
  |                                                                       |
  -----------------------------------------------------------------------*/

  /** Creates a new SomethingSub. */
  public SomethingSub() {
    /*-----------------------------------------------------------------------
    | Constructor                                                           |
    | - Set hardware options (motor directions, encoder parameters)         |
    | - Call the subsystem init() method to reset the subsystem             |
    |                                                                       |
    -----------------------------------------------------------------------*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*-----------------------------------------------------------------------
  | public void init() method                                             |
  | - Reset all of the subsystem's hardware and state variables           |
  |                                                                       |
  -----------------------------------------------------------------------*/

  /*-----------------------------------------------------------------------
  | Other subsystem methods                                               |
  | - Methods to set or get hardware values (setArmDown, isArmUp)         |
  | - Methods to manipulate the subsystem (driveStraightMm(1000))         |
  |                                                                       |
  -----------------------------------------------------------------------*/
}
