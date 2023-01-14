import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;

public class ClimberDriveSub extends CommandBase {
  private DrivetrainSub m_drivetrainSub;
  private PS4Controller m_controller;
  TalonFX m_climbArmMotor = new TalonFX(15);
  ClimbSub ClimberDriveSub = new ClimbSub();
 
  public ClimberDriveSub(PS4Controller controller, DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    m_controller = controller;
    addRequirements(drivetrainSub);
  }

  @Override
  public void execute() {
    m_drivetrainSub.tankDrive(m_controller.getLeftY(), m_controller.getRightX());
    // For the Romi template, use m_drivetrainSub.arcadeDrive
  }

  @Override
  public void end(boolean interrupted) {
    ClimberDriveSub.m_setClimberArmPower(0.3);
  }
}

