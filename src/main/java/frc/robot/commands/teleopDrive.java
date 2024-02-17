package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveBase;

public class teleopDrive extends Command {
  private SwerveDriveBase s_swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  
  public teleopDrive(
      SwerveDriveBase s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    
    this.s_swerve = s_Swerve;
    addRequirements(s_swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values and Deadband */
    double translation = translationLimiter.calculate(
      MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.OperatorConstants.kDriverDeadband)
    );

    double strafe = strafeLimiter.calculate(
      MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.OperatorConstants.kDriverDeadband)
    );

    double rotation = rotationLimiter.calculate(
      MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.OperatorConstants.kDriverDeadband)
    );

    s_swerve.drive(
      new Translation2d(translation, strafe).times(0.5),
      rotation * 0.5, 
      !robotCentricSup.getAsBoolean(),
      true
    );
  }
}
