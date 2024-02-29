package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;


public class RobotContainer {
  /* ROBOT SUBSYSTEM DEFINITIONS */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* SET DRIVER CONTROLLER OBJECTS */
  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverUSBPort);
  // final CommandXboxController m_actionXbox = new CommandXboxController(OperatorConstants.kActionUSBPort);


  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /* CONFIGURE PRE-DEFINED COMMANDS */
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.kDriverDb_LeftX),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.kDriverDb_LeftY),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.kDriverDb_RightX),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);
    /* Applies deadbands and inverts controls because joysticks are
     * back-right positive while the robot controls are front-left positive.
     * LEFT STICK controls translation
     * RIGHT STICK controls the desired angle, NOT angular rotation.
     */
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.kDriverDb_LeftY),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.kDriverDb_LeftX),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    /* Applies deadboand and inverts controls becasue joysticks are
     * back right positive while robot controls are fonrt-left positive.
     * LEFT STICK controls translation
     * RIGHT STICK controls angular velocity of robot
     */
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.kDriverDb_LeftY),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.kDriverDb_LeftX),
        () -> driverXbox.getRightX() * 0.5);
    // Uncomment the following if you plan on using the code simulation which might not be setup.
    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    /* SET DEFAULT COMMANDS & BUTTON BINDINGS */
    // configureDefaults();
    configureDriveBindings();
    configureActionBindings();
  }


  
  /** 
   * CONFIGURE DEFAULT COMMANDS
   * Sets the default commands for a given subsystem using the .setDefaultCommand() function.
   */
  private void configureDefaults() {
    /* Drivetrain Default Command */
    // If wanting to run in simulation mode (might not be setup), swap which line is commented.
    //drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    /* Example Subsystem Default Command */
    // m_exampleSubsystem.setDefaultCommand(new ExampleCommand(m_exampleSubsystem));
  }


  
  /**
   * CONFIGURE BUTTON BINDINGS
   * Use this method to define your trigger->command mappings. Triggers can be controller button
   * presses, events that occur, or even limit switches on the robot.
   */
  private void configureDriveBindings() {
    /* Reset Gyro Angle Manually */
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    
    /* Manually Add a Fake Vision Reading??? or lock the wheels into defense mode */
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    /* DRIVE TO A SPECIFIC POSITION AT AN ANGLE OF ZERO */
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    
  }

  private void configureActionBindings() {}



  /**
   * AUTONOMOUS MODE SETTING
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
