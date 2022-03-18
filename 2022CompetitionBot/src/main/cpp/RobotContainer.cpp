// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/button/JoystickButton.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "RobotContainer.h"
#include "commands/DriveWithJoystickCmd.h"
#include "commands/KillEverythingCmd.h"
#include "commands/IntakeCargoCmd.h"
#include "commands/ToggleIntakeArmCmd.h"
#include "commands/ShootCargoCmd.h"
#include "commands/SpinFlywheelCmd.h"
#include "commands/IntakeJoystickCmd.h"
#include "commands/ShootAndTaxiGrp.h"
#include "commands/ClimberArmCmd.h"
#include "commands/ShiftLowCmd.h"
#include "commands/ShiftHighCmd.h"
#include "commands/ArmSeparationCmd.h"
#include "commands/ShiftAutoCmd.h"
#include "commands/TaxiGrp.h"
#include "commands/TwoBallAutoGrp.h"
#include "commands/AlignThenShootGrp.h"
#include "commands/FourBallAutoGrp.h"

////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

// Test that we can create all of our hardware objects.
// DO NOT leave this enabled.  Testing only.
#if 0 // Set to 1 to test, 0 for robot deployment
#include <ctre/Phoenix.h>
ctre::phoenix::motorcontrol::can::WPI_TalonSRX myCanTalonSrxMotorController(30);    // TalonSRX via CAN (ID 30)
ctre::phoenix::motorcontrol::can::WPI_VictorSPX myCanVictorSpxMotorController(29);  // VictorSPX via CAN (ID 29)
ctre::phoenix::motorcontrol::can::WPI_TalonFX myCanTalonFXMotorController(28);      // TalonFX via CAN (ID 28)

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
rev::CANSparkMax myCanSparkMaxMotorController(27, rev::CANSparkMaxLowLevel::MotorType::kBrushless); // SparkMax via CAN (ID 27) for brushless motor (e.g. Neo)

#include <rev/ColorSensorV3.h>
rev::ColorSensorV3 myColorSensor(frc::I2C::kOnboard); // Colour sensor connected via Rio's I2C port

#include <AHRS.h>
AHRS myNavX2(frc::SPI::kMXP); // NavX/NavX2 Attitude and Heading Reference System (i.e. gyroscope) connected via Rio's MXP port
#endif
//
////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

/*
 * ON LOGITECH F310 CONTROLLER:
 * X = 1            (Blue)
 * A = 2            (Green)
 * B = 3            (Red)
 * Y = 4            (Yellow)
 * LB = 5           (Left-Bumper: top button)
 * RB = 6           (Right-Bumper: top button)
 * LT = 7           (Left-Trigger: bottom button)
 * RT = 8           (Right-Trigger: bottom button)
 * Select/Back = 9  (Above left joystick)
 * Start = 10       (Above right joystick)
 * L3 = 11          (Press left joystick)
 * R3 = 12          (Press right joystick)
 * 
 * Left Joystick Vertical Axis = 1
 * Left Joystick Horizontal Axis = 0
 * Right Joystick Vertical Axis = 3
 * Right Joystick Horizontal Axis = 2
 */

//Driver Buttons

constexpr int kAlignThenShoot = 1;
constexpr int kAligntoVision = 3;
constexpr int kShiftAuto = 4;
constexpr int kShiftLowDrvBtn = 5;
constexpr int kShiftHighDrvBtn = 6;
constexpr int kKillEverythingDrv1Btn = 11;
constexpr int kKillEverythingDrv2Btn = 12;

// Operator Buttons
constexpr int kSpinFlywheelOpBtn = 1;
constexpr int kIntakeCargoOpBtn = 2;
constexpr int kToggleIntakeArmOpCmd = 4;
constexpr int kShootCargoLowOpBtn = 7;
constexpr int kShootCargoHighOpBtn = 8;
constexpr int kKillEverythingOp1Btn = 11;  // Same as driver
constexpr int kKillEverythingOp2Btn = 12;
constexpr int kClimberExtendOpBtn = 6;
constexpr int kClimberRetractOpBtn = 5;
constexpr int kArmSeparationOpBtn = 9;


RobotContainer::RobotContainer() {

  // Initialize all of your commands and subsystems here
  m_drivetrainSub.SetDefaultCommand(DriveWithJoystickCmd(&m_drivetrainSub, &m_driverController));
  m_intakeSub.SetDefaultCommand(IntakeJoystickCmd(&m_intakeSub, &m_operatorController));
  autoChooserSetup();

  // Configure the button bindings
  ConfigureButtonBindings();

  initDashboard();
}

void RobotContainer::ConfigureButtonBindings() {
  
  // Driver Controller Button Mapping
  frc2::JoystickButton AligntoVision(&m_driverController, kAligntoVision);
  AligntoVision.WhenPressed(AlignToVisionCmd(&m_drivetrainSub, &m_visionSub));

  frc2::JoystickButton shiftLowDrvBtn(&m_driverController, kShiftLowDrvBtn);
  shiftLowDrvBtn.WhenPressed(ShiftLowCmd(&m_drivetrainSub));

  frc2::JoystickButton shiftHighDrvBtn(&m_driverController, kShiftHighDrvBtn);
  shiftHighDrvBtn.WhenPressed(ShiftHighCmd(&m_drivetrainSub));

  frc2::JoystickButton shiftAuto(&m_driverController, kShiftAuto);
  shiftAuto.WhenPressed(ShiftAutoCmd(&m_drivetrainSub));

  frc2::JoystickButton killEverythingDrv1Btn(&m_driverController, kKillEverythingDrv1Btn);
  killEverythingDrv1Btn.WhenPressed(KillEverythingCmd(&m_climberSub, &m_drivetrainSub, &m_intakeSub, &m_shooterSub));

  frc2::JoystickButton killEverythingDrv2Btn(&m_driverController, kKillEverythingDrv2Btn);
  killEverythingDrv2Btn.WhenPressed(KillEverythingCmd(&m_climberSub, &m_drivetrainSub, &m_intakeSub, &m_shooterSub));

  frc2::JoystickButton AlignThenShoot(&m_driverController,kAlignThenShoot);
  AlignThenShoot.WhenPressed(AlignThenShootGrp(&m_shooterSub,&m_visionSub,&m_drivetrainSub,&m_intakeSub, false));

  
  // Operator Controller Button Mapping
  frc2::JoystickButton intakeCargoOpBtn(&m_operatorController, kIntakeCargoOpBtn); //Intake cargo
  intakeCargoOpBtn.WhileHeld(IntakeCargoCmd(&m_intakeSub));

  frc2::JoystickButton toggleIntakeArmOpBtn(&m_operatorController, kToggleIntakeArmOpCmd); //Toggle intake
  toggleIntakeArmOpBtn.WhenPressed(ToggleIntakeArmCmd(&m_intakeSub));

  frc2::JoystickButton shootCargoLowOPBtn(&m_operatorController, kShootCargoLowOpBtn); //Low cargo shoot
  shootCargoLowOPBtn.WhileHeld(ShootCargoCmd(&m_shooterSub, &m_intakeSub, &m_visionSub, false, false));  

  frc2::JoystickButton shootCargoHighOPBtn(&m_operatorController, kShootCargoHighOpBtn); //High cargo shoot
  shootCargoHighOPBtn.WhileHeld(ShootCargoCmd(&m_shooterSub, &m_intakeSub, &m_visionSub, true, false)); 
  
  frc2::JoystickButton spinFlywheelOpBtn(&m_operatorController, kSpinFlywheelOpBtn); //Spin flywheel
  spinFlywheelOpBtn.WhileHeld(SpinFlywheelCmd(&m_shooterSub, true));

  frc2::JoystickButton killEverythingOp1Btn(&m_operatorController, kKillEverythingOp1Btn);
  killEverythingOp1Btn.WhenPressed(KillEverythingCmd(&m_climberSub, &m_drivetrainSub, &m_intakeSub, &m_shooterSub));

  frc2::JoystickButton killEverythingOp2Btn(&m_operatorController, kKillEverythingOp2Btn);
  killEverythingOp2Btn.WhenPressed(KillEverythingCmd(&m_climberSub, &m_drivetrainSub, &m_intakeSub, &m_shooterSub));

  frc2::JoystickButton climberExtendOpBtn(&m_operatorController, kClimberExtendOpBtn); //raise climber
  climberExtendOpBtn.WhileHeld(ClimberArmCmd(&m_climberSub, true));
  
  frc2::JoystickButton climberRetractOpBtn(&m_operatorController, kClimberRetractOpBtn); // retract climber
  climberRetractOpBtn.WhileHeld(ClimberArmCmd(&m_climberSub, false));

  frc2::JoystickButton ArmSeparationOpBtn(&m_operatorController, kArmSeparationOpBtn); // arm separation cmd
  ArmSeparationOpBtn.WhenPressed(ArmSeparationCmd(&m_climberSub,true ));

  // Axis mapping
  m_driverController.SetXChannel(0);
  m_driverController.SetYChannel(1);
  m_driverController.SetZChannel(2);
  m_driverController.SetThrottleChannel(3);

  m_operatorController.SetXChannel(0);
  m_operatorController.SetYChannel(1);
  m_operatorController.SetZChannel(2);
  m_operatorController.SetThrottleChannel(3);
}

void RobotContainer::initSubsystems() { 
  m_drivetrainSub.init(); 
  m_shooterSub.init();
  m_climberSub.init();
  m_intakeSub.init(); 
  m_visionSub.init();
}

void RobotContainer::initDashboard(){
  frc::SmartDashboard::PutNumber("Low Speed", m_shooterSub.m_lowerBinSpeed);
  frc::SmartDashboard::PutNumber("High Speed", m_shooterSub.m_upperBinSpeed);
    //Mess up current shooter speed
  //frc::SmartDashboard::PutNumber("Shoot kF", m_shooterSub.m_kNewF);
  //frc::SmartDashboard::PutNumber("Shoot kP", m_shooterSub.m_kNewP);
  //frc::SmartDashboard::PutNumber("Shoot kD", m_shooterSub.m_kNewD);
  //frc::SmartDashboard::PutNumber("Shoot kI", m_shooterSub.m_kNewI);
}

void RobotContainer::updateDashboard() {
  m_shooterSub.m_lowerBinSpeed = frc::SmartDashboard::GetNumber("Low Speed", m_shooterSub.m_lowerBinSpeed);
  m_shooterSub.m_upperBinSpeed = frc::SmartDashboard::GetNumber("High Speed", m_shooterSub.m_upperBinSpeed);
  frc::SmartDashboard::PutNumber("Speed", (m_drivetrainSub.getLeftVelocity()+ m_drivetrainSub.getRightVelocity()) / 2);
  frc::SmartDashboard::PutNumber("Flywheel Speed", m_shooterSub.getSpeed());
  frc::SmartDashboard::PutNumber("Climb Arm", m_climberSub.getClimberEncoder());
  frc::SmartDashboard::PutNumber("Climber Height", m_climberSub.getClimberEncoder());
  frc::SmartDashboard::PutNumber("Drive Left", m_drivetrainSub.getLeftEncoderDistanceM());
  frc::SmartDashboard::PutNumber("Drive Right", m_drivetrainSub.getRightEncoderDistanceM());
  frc::SmartDashboard::PutNumber("Heading", m_drivetrainSub.getHeading());
  frc::SmartDashboard::PutNumber("Horizontal Angle", m_visionSub.getHorizontalAngle());
  frc::SmartDashboard::PutBoolean("Auto Shift", m_drivetrainSub.m_isAutoShift);
  frc::SmartDashboard::PutBoolean("is High Gear", m_drivetrainSub.isShiftedInHighGear());
  frc::SmartDashboard::PutBoolean("Front Magazine", m_intakeSub.isCargoAtMagazineFront());
  frc::SmartDashboard::PutBoolean("Back Magazine", m_intakeSub.isCargoAtMagazineBack());
  frc::SmartDashboard::PutBoolean("Intake End", m_intakeSub.isCargoAtIntakeEnd());
  //Mess up current shooter speed
  //m_shooterSub.m_kNewF = frc::SmartDashboard::GetNumber("Shoot kF", m_shooterSub.m_kNewF);
  //m_shooterSub.m_kNewP = frc::SmartDashboard::GetNumber("Shoot kP", m_shooterSub.m_kNewP);
  //m_shooterSub.m_kNewD = frc::SmartDashboard::GetNumber("Shoot kD", m_shooterSub.m_kNewD);
  //m_shooterSub.m_kNewI = frc::SmartDashboard::GetNumber("Shoot kI", m_shooterSub.m_kNewI);
}

void RobotContainer::autoChooserSetup() {
  m_autoChooser.SetDefaultOption("Do nothing", new AutoDoNothingCmd());  
  m_autoChooser.AddOption("Shoot and Taxi", new ShootAndTaxiGrp(&m_shooterSub, &m_intakeSub, &m_drivetrainSub, &m_visionSub));  
  m_autoChooser.AddOption("Taxi", new TaxiGrp(&m_drivetrainSub));
  m_autoChooser.AddOption("Two Ball Auto", new TwoBallAutoGrp(&m_drivetrainSub, &m_intakeSub, &m_shooterSub, &m_visionSub));
  m_autoChooser.AddOption("Four Ball Auto", new FourBallAutoGrp(&m_shooterSub, &m_intakeSub, &m_drivetrainSub, &m_visionSub));

  frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_autoChooser.GetSelected();
}
