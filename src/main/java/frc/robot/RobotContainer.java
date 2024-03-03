package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Handler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;


public class RobotContainer {
  /* ROBOT SUBSYSTEM DEFINITIONS */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  private final Climb climb = new Climb();
  private final Handler tilt = new Handler();
  private final Launcher launch = new Launcher();
  private final Intake intake = new Intake();

  /* SET DRIVER CONTROLLER OBJECTS */
  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverUSBPort);
  private final CommandXboxController m_actionXbox = new CommandXboxController(OperatorConstants.kActionUSBPort);

  /* ARBITRARY TRIGGERS */
  private final Trigger climbResetTrigger = new Trigger(climb::getLowerProx);
  private final Trigger climbTopTrigger = new Trigger(climb::getHigherProx);
  private final Trigger intakeNoteTrigger = new Trigger(intake::getNoteProx);
  // private final Trigger intakeMovingTrigger = new Trigger(intake::isMoving);
  //private final Trigger tiltHighLimitTrigger = new Trigger(tilt::atHighRage);
  //private final Trigger tiltLowLimitTrigger = new Trigger(tilt::atLowRage);

  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();


  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    tilt.disable();

    m_AutoChooser.setDefaultOption("None", Autos.none());
    //m_AutoChooser.addOption("Drive_Straight", Autos.driveLine(drivebase));

    SmartDashboard.putData(m_AutoChooser);
    
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
     */ // JONAH PREFERS THIS DRIVING METHOD
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
    // intake.setDefaultCommand(new InstantCommand(intake::Stop, intake));
    // climb.setDefaultCommand(new InstantCommand(climb::Stop, climb));
    Command manualTilt = tilt.manualTilt(() -> m_actionXbox.getLeftY());
    tilt.setDefaultCommand(manualTilt);
  }


  
  /**
   * CONFIGURE BUTTON BINDINGS
   * Use this method to define your trigger->command mappings. Triggers can be controller button
   * presses, events that occur, or even limit switches on the robot.
   */
  private void configureDriveBindings() {
    /* Y Button - Reset Gyro Angle Manually */
    driverXbox.y()
        .debounce(0.1, Debouncer.DebounceType.kBoth)                    // Prevents rapid repeated triggering
        .onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));                 // When button changes from false to true, trigger command once.
    
    /* X Button - Lock Wheels in X-Mode */
    driverXbox.x()
          .debounce(0.1, Debouncer.DebounceType.kBoth)    // Prevents rapid repeated triggering
          .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());  // Will keep wheels in locked position so long as the button is held.
                     
    /* CLIMB CONTROLS */
    /* Left Bumber - Climber Column Down */
    driverXbox.leftBumper()
        //.debounce(0.5, Debouncer.DebounceType.kBoth)                           // Prevents rapid repeated triggering
        .whileTrue(
            Commands.startEnd(climb::Down, climb::Stop, climb)                                   // Run climber column down
                    .until(climbResetTrigger)                                  // Until while true ends or if the switch is hit.           
        );
    /* Left Bumber & A Button - Climber Column Down (No Limits) */
    // driverXbox.leftBumper().and(driverXbox.a())
    //     .debounce(0.5, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
    //     .whileTrue(
    //       Commands.run(climb::Down, climb)                            // Run climber down
    //               .andThen(Commands.runOnce(climb::Stop, climb))    // Stop climber after interrupted
    //     );
    /* Right Bumber - Climber Column Up */
    driverXbox.rightBumper()
        //.debounce(0.5, Debouncer.DebounceType.kBoth)                // Prevents rapid repeated triggering
        .whileTrue(
          Commands.startEnd(climb::Up, climb::Stop, climb)                           // Run climber column up
                  .until(climbTopTrigger)                           // Until while true ends or if the switch is hit // Stop the climb motor
        );
    // /* Right Bumber & A Button - Climber Column Up (No Limits) */
    // driverXbox.rightBumper().and(driverXbox.a())
    //     .debounce(0.5, Debouncer.DebounceType.kBoth)               // Prevents rapid repeated triggering
    //     .whileTrue(
    //       Commands.run(climb::Up, climb)                           // Run climber up
    //               .andThen(Commands.runOnce(climb::Stop, climb)) // Stop climber after interrupted
    //     );
    
    // driverXbox.rightBumper().whileTrue(new StartEndCommand(
    //   climb::Up,
    //   climb::Stop,
    //   climb
    // ));

    // driverXbox.rightBumper().whileTrue(new StartEndCommand(
    //   climb::Down,
    //   climb::Stop,
    //   climb
    // ));
  }

  private void configureActionBindings() {
    /** INTAKE BUTTONS */
    /* B Button - Intake Spinning Inwards */
    m_actionXbox.b()               // When B is pressed but Start is not pressed
        .debounce(0.2, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
        .whileTrue(
          Commands.parallel(
              Commands.startEnd(intake::In, intake::Stop, intake),                            // Run the intake inwards until let go or switch is triggered
              Commands.startEnd(()->launch.load(0.4), launch::stop, launch)
            )                    // Stop the intake after the command ends.
        );
    // OR use this where the intake wheel moving will trigger the command to bring the note in
    // intakeMovingTrigger.debounce(1.0, Debouncer.DebounceType.kBoth)
    //     .onTrue(
    //       Commands.run(intake::In, intake)                            // Run the intake wheel inwards towards the robot
    //               .onlyIf(intakeNoteTrigger.negate())                 // Only run if the note prox switch is not (negate) triggered
    //               .until(intakeNoteTrigger)                           // Run until the note prox switch is triggered
    //               .andThen(new WaitCommand(0.5))                      // Wait 1/2 second so the note has time to get in the robot
    //               .andThen(Commands.runOnce(intake::Stop))            // Stop the intake after the command ends.
    //     );
    
    /* B & Start Buttons - Intake Spinning Inwards (No Limits) */
    // m_actionXbox.b().and(m_actionXbox.start())                        // When X and Start are both pressed
    //     .debounce(0.2, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
    //     .whileTrue(
    //       Commands.run(intake::In, intake)                            // Run the intake inwards until let go or switch is triggered
    //               .andThen(Commands.runOnce(intake::Stop, intake))  // Stop the intake after the command ends.
    //     );
    /* Y Button - Intake Spinning Outwards */
    // m_actionXbox.y()                                                  // When Y is pressed
    //     .debounce(0.4, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
    //     .whileTrue(
    //       Commands.startEnd(intake::Out, intake::Stop, intake)                           // Run the intake inwards until let go or switch is triggered
    //     );
    // m_actionXbox.b().whileTrue(new runIntake(intake)).onFalse(new InstantCommand(intake::Stop, intake));
    // m_actionXbox.start().whileTrue(
    //   new InstantCommand(
    //     intake::Out,
    //     intake
    //   ));

    /** HANDLE BINDINGS */
    /* POV Center - Tilter Stops Moving */
    // Bad idea since it will stop the tilter PID from finishing

    m_actionXbox.leftBumper()
        .debounce(0.2, Debouncer.DebounceType.kBoth)
        .whileTrue(
          Commands.startEnd(
          tilt::manualDown, tilt::stop, tilt)
          .unless(climbResetTrigger)
        );
    
    m_actionXbox.rightBumper()
        .debounce(0.2, Debouncer.DebounceType.kBoth)
        .whileTrue(
          Commands.startEnd(
          tilt::manualUp, tilt::stop, tilt)
        );
    
    

    /* POV Up - Tilter Move to Speaker Position */
    m_actionXbox.povUp()
        .onTrue(
          Commands.sequence(
              Commands.sequence(
                    Commands.runOnce(() -> {tilt.enable();}, tilt),
                    Commands.runOnce(() -> {tilt.setSetpoint(Constants.TiltAngles.kSpeakerAngle);}, tilt))
                  .withTimeout(10.0),
              Commands.runOnce(() -> {tilt.disable();}, tilt),
                    Commands.runOnce(tilt::stop, tilt))
                    
        );
    
    // // /* POV Right - Tilter Move to Amp Position */
    // m_actionXbox.povRight()
    //     .debounce(0.4, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
    //     .onTrue(
    //       Commands.sequence(
    //                 Commands.runOnce(() -> {tilt.setSetpoint(Constants.TiltAngles.kAmpAngle);}, tilt),
    //                 Commands.runOnce(() -> {tilt.enable();}, tilt))
    //               .withTimeout(10.0)
    //               .andThen(Commands.sequence(
    //                 Commands.runOnce(() -> {tilt.disable();}, tilt),
    //                 Commands.runOnce(tilt::stop, tilt)))
    //     );
    
    // // /* POV Down - Tilter Move to Floor Position */
    // m_actionXbox.povDown()
    //     .debounce(0.4, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
    //     .onTrue(
    //       Commands.sequence(
    //                 Commands.runOnce(() -> {tilt.setSetpoint(Constants.TiltAngles.kFloorAngle);}, tilt),
    //                 Commands.runOnce(() -> {tilt.enable();}, tilt))
    //               .withTimeout(10.0)
    //               .andThen(Commands.sequence(
    //                 Commands.runOnce(() -> {tilt.disable();}, tilt),
    //                 Commands.runOnce(tilt::stop, tilt)))
    //     );

    /** LAUNCHER BINDINGS */
    /* Select Button - Manual Feed Solenoid */
    m_actionXbox.a()  // When Select is pressed
        // .debounce(0.4, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
        .whileTrue(
          Commands.startEnd(
            tilt::feedExtend,
            tilt::feedRetract,
            tilt));
    
    /* X Button - Manual Launcher Spin Up */
    m_actionXbox.x()  // When X is pressed ans Start is not
        .debounce(0.4, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
        .whileTrue(
          Commands.startEnd(
            () -> launch.launch(0.85),
            launch::stop,
            launch
          )
        );
    
    /* X & Start Buttons - Manual Launcher Spin Reverse */
    // m_actionXbox.x()  // When X and Start are both pressed
    //     .debounce(0.4, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
    //     .whileTrue(
    //       Commands.startEnd(
    //         () -> launch.launch(-0.5),
    //         launch::stop,
    //         launch
    //       )
    //     );
    
    /* A Button - Launching Sequence */
    // m_actionXbox.a()
    //     .debounce(0.4, Debouncer.DebounceType.kBoth)                  // Prevents rapid repeated triggering
    //     .onTrue(
    //       Commands.sequence(
    //         Commands.runOnce(() -> launch.load(0.5), launch),              // Move note away from launch wheels
    //         Commands.waitSeconds(0.5),                                     // Hold condition for 0.4 seconds
    //         Commands.runOnce(launch::stop, launch),                        // Stop launch wheels
    //         Commands.waitSeconds(0.5),                                     // Hold condition for 0.2 seconds
    //         Commands.parallel(
    //           Commands.runOnce(() -> launch.setLaunchRPM(4000), launch),   // Set RPM PID to 4000 RPM
    //           Commands.runOnce(tilt::feedExtend, tilt)                     // Feed note into launcher after 2.0 seconds of warm up
    //                   .beforeStarting(Commands.waitSeconds(2.0))),
    //         Commands.parallel(
    //           Commands.runOnce(launch::stop, launch),                      // Turn everything off
    //           Commands.runOnce(tilt::feedRetract, tilt))
    //       )
    //     );
    
  }



  /**
   * AUTONOMOUS MODE SETTING
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Commands.none();
    return m_AutoChooser.getSelected();
    
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
