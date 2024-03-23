package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Handler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Solenoid;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


public class RobotContainer {
  /* ROBOT SUBSYSTEM DEFINITIONS */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  private final Climb climb = new Climb();
  private final Handler tilt = new Handler();
  private final Launcher launch = new Launcher();
  private final Intake intake = new Intake();
  private final Solenoid sol = new Solenoid();

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

    NamedCommands.registerCommand("Shoot", Autos.shoot(climb, tilt, launch, sol));
    NamedCommands.registerCommand("Shoot-Preset", Autos.shootPreset(launch, sol));
    NamedCommands.registerCommand("Tilt Down", Commands.startEnd(tilt::manualDown, tilt::stop, tilt).withTimeout(4.0));
    NamedCommands.registerCommand("Intake In", Commands.runOnce(intake::In, intake));
    NamedCommands.registerCommand("Intake Off", Commands.runOnce(intake::Stop, intake));
    m_AutoChooser = AutoBuilder.buildAutoChooser();

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
    /* DRIVE CONTROLS */
    /* Y Button - Reset Gyro Angle Manually */
    driverXbox.y()
        .onTrue(Commands.runOnce(drivebase::zeroGyro, drivebase));                 // When button changes from false to true, trigger command once.
    
    /* X Button - Lock Wheels in X-Mode */
    driverXbox.x()
          .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());  // Will keep wheels in locked position so long as the button is held.
                     
    /* CLIMB CONTROLS */
    /* Left Bumber - Climber Column Down */
    driverXbox.leftBumper()
        .whileTrue(
            Commands.startEnd(climb::Down, climb::Stop, climb)                                   // Run climber column down
                    .until(climbResetTrigger)                                  // Until while true ends or if the switch is hit.           
        );

    driverXbox.back().and(driverXbox.leftBumper())
        .whileTrue(
          Commands.startEnd(climb::Down, climb::Stop, climb)
        );

    /* A Button - Climber Up to the Speaker Shootinh Height */
    driverXbox.a()
        .onTrue(
          Commands.sequence(
            Commands.runOnce(climb::Up),
            Commands.waitUntil(climb::atSpeakerHeight).withTimeout(10.0),
            Commands.runOnce(climb::Stop, climb)
          )
        );

    driverXbox.start()
        .onTrue(
          Commands.sequence(
            Commands.runOnce(climb::Down),
            Commands.waitUntil(climb::atStageHeight).withTimeout(6.0),
            Commands.runOnce(climb::Stop, climb)
          )
        );

    /* B Button - Climber Up to Max Position Controlled by Limit Switch */
    driverXbox.b()
        .onTrue(
          Commands.sequence(
            Commands.runOnce(climb::Up, climb),
            Commands.waitUntil(climbTopTrigger).withTimeout(20.0),
            Commands.runOnce(climb::Stop, climb)
        ));

    /* Right Bumber - Climber Column Up */
    driverXbox.rightBumper()
        .whileTrue(
          Commands.startEnd(climb::Up, climb::Stop, climb)                           // Run climber column up
                  .until(climbTopTrigger)                           // Until while true ends or if the switch is hit // Stop the climb motor
        );
    
    /* Right Bumber & A Button - Climber Column Up (No Limits) */
    // driverXbox.rightBumper().and(driverXbox.a())
    //     .whileTrue(
    //       Commands.startEnd(climb::Up, climb::Stop, climb)                           // Run climber up
    //     );
  }



  
  private void configureActionBindings() {
    /** INTAKE BUTTONS */
    /* B Button - Intake Spinning Inwards */
    m_actionXbox.b()               // When B is pressed but Start is not pressed
        .whileTrue(
          Commands.parallel(
              Commands.startEnd(intake::In, intake::Stop, intake),                            // Run the intake inwards until let go
              Commands.startEnd(()->launch.load(0.45), launch::stop, launch)
            )                    // Stop the intake after the command ends.
        );

    /* B Button + Start Button - Intake Spinning Outwards */
    // m_actionXbox.b().and(m_actionXbox.start())               // When B is pressed and Start is pressed
    //     .whileTrue(
    //       Commands.parallel(
    //           Commands.startEnd(intake::Out, intake::Stop, intake),                            // Run the intake outwards until let go
    //           Commands.startEnd(()->launch.unload(0.2), launch::stop, launch)
    //         )                    // Stop the intake after the command ends.
    //     );

    /** HANDLE BINDINGS */
    /* Left Bumper - Manual Tilt Down Button (in Case Override is Needed) */
    m_actionXbox.leftBumper()
        .whileTrue(
          Commands.startEnd(
          tilt::manualDown, tilt::stop, tilt)
          .beforeStarting(() -> tilt.disable(), tilt)
        );

    /* Right Bumper - Manual Tilt Up Button (in Case Ovveride is Needed) */
    m_actionXbox.rightBumper()
        .whileTrue(
          Commands.startEnd(
          tilt::manualUp, tilt::stop, tilt)
          .beforeStarting(() -> tilt.disable(), tilt)
        );
    
    /** LAUNCHER BINDINGS */
    /* A Button - Manual Feed Solenoid */
    m_actionXbox.a()  // When Select is pressed
        .whileTrue(
          Commands.startEnd(
            sol::feedExtend,
            sol::feedRetract,
            sol));
    
    /* X Button - Manual Launcher Spin Up */
    m_actionXbox.x()  // When X is pressed
        .whileTrue(
          Commands.startEnd(
            () -> launch.launch(frc.robot.Constants.Launcher.kSpeedPushPercent),
            launch::stop,
            launch
          )
        );

    /* Y Button - Launch Note Sequence */
    m_actionXbox.y()
        .onTrue(
          Commands.sequence(
            Commands.startEnd(
              () -> launch.load(frc.robot.Constants.Launcher.kSpeedPull), 
              () -> launch.launch(frc.robot.Constants.Launcher.kSpeedPushPercent), //launch.setLaunchRPM(3500), 
              launch
            ).withTimeout(0.4),
            Commands.waitSeconds(0.8),
            Commands.startEnd(
              sol::feedExtend, 
              sol::feedRetract, 
              sol
            ).withTimeout(1.0),
            Commands.runOnce(launch::stop, launch).beforeStarting(Commands.waitSeconds(0.6))
          )
        );

    /* POV Buttons - Setpoints for Amp, Speaker, and Pickup */
    m_actionXbox.povRight()
        .onTrue(
          Commands.runOnce(
            () -> {
              tilt.setGoal(Constants.Handler.kAmpPosition);
              tilt.enable();
            },
            tilt));
    m_actionXbox.povUp()
        .onTrue(
          Commands.runOnce(
            () -> {
              tilt.setGoal(Constants.Handler.kTiltMaxAngle);
              tilt.enable();
            },
            tilt));
    m_actionXbox.povDown()
        .onTrue(
          Commands.runOnce(
            () -> {
              tilt.setGoal(Constants.Handler.kTiltMinAngle);
              tilt.enable();
            },
            tilt));
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

  /**
   * DRIVE MOTOR BRAKE
   * Set the brake mode on the drive motors.
   * @param brake is enabled (true) or coast is enabled (false)
   */
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
