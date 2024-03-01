package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.runIntake;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Handler;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;


public class RobotContainer {
  /* ROBOT SUBSYSTEM DEFINITIONS */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  final Climb climb = new Climb();
  private final Handler tilt = new Handler();
  // private final Launcher m_launch = new Launcher();
  private final Intake intake = new Intake();
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* SET DRIVER CONTROLLER OBJECTS */
  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverUSBPort);
  final CommandXboxController m_actionXbox = new CommandXboxController(OperatorConstants.kActionUSBPort);


  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /* CONFIGURE PRE-DEFINED COMMANDS */

    /* Applies deadbands and inverts controls because joysticks are
     * back-right positive while th erobot controls are font-left positive.
     * LEFT STICK controls translation
     * RIGHT STICK controls heading velocity
     */
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
        () -> driverXbox.getRightX() * 0.75);
    // Uncomment the following if you plan on using the code simulation which might not be setup.
    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    /* SET DEFAULT COMMANDS & BUTTON BINDINGS */
    configureDefaults();
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

    // climb.setDefaultCommand(new InstantCommand(climb::Stop, climb));
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

  private void configureActionBindings() {
    /** CLIMB BUTTONS */
    m_actionXbox.povCenter().onTrue(
      new InstantCommand(
        climb::Stop,
        climb
    ));

    m_actionXbox.povUp().onTrue( // UP
      new InstantCommand(
        climb::Up,
        climb
    ));

    m_actionXbox.povDown().onTrue( // DOWN
      new InstantCommand(
        climb::Down,
        climb
    ));

    m_actionXbox.povRight().onTrue( // Up to position
      new InstantCommand(
        () -> climb.setPosition(Constants.Climb.kChainHeight),
        climb
      ).withTimeout(30.0));

    m_actionXbox.povLeft().onTrue( // Down to zero
      new InstantCommand(
        () -> climb.setPosition(11.0),
        climb
      ));

    Trigger climbResetTrigger = new Trigger(climb::getLowerProx);
    climbResetTrigger.onTrue(new InstantCommand(climb::resetPosition, climb));

    Trigger climbStopTrigger = new Trigger(climb::getHigherProx);
    climbStopTrigger.onTrue(new InstantCommand(climb::Stop, climb));

    /** INTAKE BUTTONS */
    // m_actionXbox.b().whileTrue(
      
    //   new  (
    //     () -> intake.In(m_actionXbox.back().getAsBoolean()),
    //     intake
    //   ));
    m_actionXbox.b().whileTrue(new runIntake(intake)).onFalse(new InstantCommand(intake::Stop, intake));

    m_actionXbox.start().whileTrue(
      new InstantCommand(
        intake::Out,
        intake
      ));

    /** HANDLE BINDINGS */
    m_actionXbox.rightBumper().whileTrue(
      new InstantCommand(
        tilt::manualUp,
        tilt
      ).andThen(new InstantCommand(tilt::stop, tilt)));

    m_actionXbox.leftBumper().whileTrue(
      new InstantCommand(
        tilt::manualDown,
        tilt
      ).andThen(new InstantCommand(tilt::stop, tilt)));
  }



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
