package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveBase;

public class teleopDrive extends Command {
  private final SwerveDriveBase m_drive;
  private final CommandXboxController m_controller;
  
  public teleopDrive(SwerveDriveBase drive, CommandXboxController controller) {
    m_drive = drive;
    m_controller = controller;
    addRequirements(m_drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double desiredTrans[] = MathUtils.inputTransform(-m_controller.getLeftY(), -m_controller.getLeftX());
    double maxLinear = Constants.Swerve.kMaxDriveMeterPerSec;

    // desiredTrans[0] *= maxLinear;
    // desiredTrans[1] *= maxLinear;

    double xTrans = -m_controller.getLeftY() * maxLinear;
    double yTrans = -m_controller.getLeftX() * maxLinear;
    double rot = m_controller.getRightX() * Constants.Swerve.kMaxTurnRadianPerSec;

    // double desiredRot = -MathUtils.inputTransform(m_controller.getRightX()) * Constants.Swerve.kMaxTurnRadianPerSec;

    m_drive.drive(xTrans, yTrans, rot, false, false);
    // m_drive.drive(desiredTrans[0], desiredTrans[1], desiredRot, false, false);

    SmartDashboard.putBoolean("DrivingByController", true);
  }
}
