// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class CanIds {
        // Can ids
    public final static int kDriveMotorFL = 1; // Front left motor
    public final static int kSteeringMotorFL = 2;

    public final static int kDriveMotorFR = 3; // Front right motor
    public final static int kSteeringMotorFR = 4;

    public final static int kDriveMotorBL = 5; // Back left motor
    public final static int kSteeringMotorBL = 6;

    public final static int kDriveMotorBR = 7; // Back right motor
    public final static int kSteeringMotorBR = 8;

    public final static int kEncoderFL = 0; // All steering encoders

    public final static int kEncoderFR = 0;

    public final static int kEncoderBL = 0;
    
    public final static int kEncoderBR = 0;
    }
}
