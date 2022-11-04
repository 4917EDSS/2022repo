import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;
 
public class DriveJoystickCmd extends CommandBase {
  DrivetrainSub m_drivetrainSub;
  PS4Controller m_controller;
 
  public DriveJoystickCmd(PS4Controller controller, DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    m_controller = controller;
    addRequirements(drivetrainSub);
  }
  
  @Override
  public void execute() {
    m_drivetrainSub.tankDrive(m_controller.getLeftY(), m_controller.getRightY());
    // For the Romi template, use m_drivetrainSub.arcadeDrive
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.arcadeDrive(0.0, 0.0);
  }
