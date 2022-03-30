// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace CanIds {
    // in order of CAN Ids
    constexpr int kLeftMotor1 = 1;
    constexpr int kLeftMotor2 = 2;
    constexpr int kLeftMotor3 = 3;
    //constexpr int kLeftMotor4 = 4;
    constexpr int kRightMotor1 = 5;
    constexpr int kRightMotor2 = 6;
    constexpr int kRightMotor3 = 7;
    //constexpr int kRightMotor4 = 8;
    constexpr int kFrontRollerIntakeMotor = 10;
    constexpr int kMagazineMotor = 11;
    constexpr int kShootMotor1 = 12;
    // constexpr int k = 13;
    // constexpr int k = 14;
    constexpr int kClimberArmbMotor = 15;
    constexpr int kShootMotor2 = 16;
}

namespace PneumaticIds {
    constexpr int kShifter = 1;
    // constexpr int k = 2;
    constexpr int kArm1 = 3;
    constexpr int kArmSeparation = 4;
}

namespace DioIds{
    constexpr int kMagazineFrontSensor = 0;
    constexpr int kMagazineTopSensor = 1;
    constexpr int kEndOfIntake = 2;
    // constexpr int k = 3;
    constexpr int kEndOfIntakeTwo = 4;
    constexpr int kMagazineFrontSensorTwo = 5;
    
} 
