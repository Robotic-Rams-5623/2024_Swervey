// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriveBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDriveBase m_drive = new SwerveDriveBase();

  /**
   * SET DRIVER CONTROLLER OBJECTS
   */
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverUSBPort);
  /*private final CommandXboxController m_actionController = 
      new CommandXboxController(OperatorConstants.kActionUSBPort);*/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    /**
     * DEFAULT COMMANDS
     * Set the default commands for the various subsystems. (i.e. Human control
     * for the robot driving)
     */
    m_drive.setDefaultCommand(
      new RunCommand(
        // Left stick X and Y controls the translation of the robot along the
        // x-axis and y-axis. Rotational control is done by the x-axis of the
        // right stick.
        () -> m_drive.drive(
          // xSpeed, ySpeed, Rotation, feild relative, ratelimited
          -MathUtil.applyDeadband(m_driverController.getLeftY(), Constants.OperatorConstants.kDriverDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), Constants.OperatorConstants.kDriverDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), Constants.OperatorConstants.kDriverDeadband),
          true, true),
          m_drive)
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /**
     * DRIVER CONTROLER BINDINGS
     */
    m_driverController.rightBumper().onTrue(
      new RunCommand(
        () -> m_drive.setPositionX(),
        m_drive)
    );
  }



  /**
   * AUTONOMOUS MODE SETTING
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
