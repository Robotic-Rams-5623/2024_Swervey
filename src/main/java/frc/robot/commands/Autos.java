package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Handler;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Solenoid;
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
   * Drive Straight Out of Starting Zone--
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

  public static Command shoot(Climb climb, Handler tilt, Launcher launch, Solenoid sol) {
    return Commands.sequence(
      Commands.runOnce(climb::Up),
      Commands.waitUntil(climb::atSpeakerHeight).withTimeout(10.0),
      Commands.parallel(
        Commands.runOnce(climb::Stop, climb),
        Commands.startEnd(tilt::manualUp, tilt::stop, tilt)
          .withTimeout(2.5)
      ),
      shootPreset(launch, sol)
    );
  }

  public static Command shootPreset(Launcher launch, Solenoid sol) {
    return Commands.sequence(
      Commands.startEnd(
        () -> launch.load(frc.robot.Constants.Launcher.kSpeedPull), 
        () -> launch.launch(frc.robot.Constants.Launcher.kSpeedPushPercent), //launch.setLaunchRPM(3500), 
        launch
      ).withTimeout(0.3),
      Commands.waitSeconds(0.5),
      Commands.startEnd(
        sol::feedExtend, 
        sol::feedRetract, 
        sol
      ).withTimeout(0.4),
      Commands.runOnce(launch::stop, launch).beforeStarting(Commands.waitSeconds(0.4))
    );
  }
}
