// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  /* ROBOT SUBSYSTEM DEFINITIONS */
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final SwerveDriveBase m_drive = new SwerveDriveBase();

  /* SET DRIVER CONTROLLER OBJECTS */
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverUSBPort);
  // private final CommandXboxController m_actionController = 
  //     new CommandXboxController(OperatorConstants.kActionUSBPort);

  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /* SET DEFAULT COMMANDS */
    configureDefaults();
    
    /* CONFIGURE BUTTON BINDINGS */
    configureBindings();
  }


  
  /** 
   * CONFIGURE DEFAULT COMMANDS
   * Sets the default commands for a given subsystem using the .setDefaultCommand() function.
   */
  private void configureDefaults() {
    /* Drivetrain Default Command */
    // m_drive.setDefaultCommand(new teleopDrive(m_drive, m_driverController));

    /* Example Subsystem Default Command */
    // m_exampleSubsystem.setDefaultCommand(new ExampleCommand(m_exampleSubsystem));
  }


  
  /**
   * CONFIGURE BUTTON BINDINGS
   * Use this method to define your trigger->command mappings. Triggers can be controller button
   * presses, events that occur, or even limit switches on the robot.
   */
  private void configureBindings() {
    /* CONTROLLER COMMAND */
    // END CONTROLLER COMMAND

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
