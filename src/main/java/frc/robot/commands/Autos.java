package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Handler;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public final class Autos {
  /** 
   * Default Autonomous Mode
   * Does nothing. The robot just sits there and contemplates life.
   */
  public static Command none() {
    return Commands.none();
  }

  /** 
   * Drive Straight Out of Starting Zone
   * Doesnt matter where you place the robot, it will just drive straight
   * out of the starting zone. This may or may not be running backwards currently...
   */
  public static Command straight() {
    return new PathPlannerAuto("Straight Auto");
  }

  /**
   * Testing Paths and Event Structuring
   * This could be anything. Whatever fancy auto we are trying out right now
   * is whatever this is probably set to. When not in use for testing it is
    * best to swap comment lines with the Commands.none() so nothing bad
    * happens on accident.
   */
  public static Command testAuto() {
    return new PathPlannerAuto("Test Auto");
    // return Commands.none();
  }
}
