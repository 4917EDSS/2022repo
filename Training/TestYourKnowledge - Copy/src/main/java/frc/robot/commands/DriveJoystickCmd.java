import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
 
public class DriveJoystickCmd extends CommandBase {
  RomiDrivetrain m_drivetrainSub;
  PS4Controller m_controller;
 
  public DriveJoystickCmd(PS4Controller controller, RomiDrivetrain drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    m_controller = controller;
    addRequirements(drivetrainSub);
  }
  @Override
  public void execute() {
    m_drivetrainSub.arcadeDrive(m_controller.getLeftY(), m_controller.getRightY());
    // For the Romi template, use m_drivetrainSub.arcadeDrive
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.arcadeDrive(0.0, 0.0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
