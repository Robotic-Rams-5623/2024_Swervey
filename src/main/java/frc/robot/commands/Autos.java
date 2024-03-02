// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  /** LAUNCH NOTE INTO SPEAKER **/
  public static Command noteSpeaker(Launcher launch, Handler tilt) {
    return Commands.sequence(
              Commands.sequence(
                  Commands.runOnce(() -> {tilt.setSetpoint(Constants.TiltAngles.kSpeakerAngle);}, tilt),
                  Commands.runOnce(() -> {tilt.enable();}, tilt))
              .withTimeout(10.0)
              .andThen(Commands.sequence(
                          Commands.runOnce(() -> {tilt.disable();}, tilt),
                          Commands.runOnce(tilt::stop, tilt)),
              Commands.sequence(
                Commands.runOnce(() -> launch.load(0.6), launch),              // Move note away from launch wheels
                Commands.waitSeconds(0.4),                                     // Hold condition for 0.4 seconds
                Commands.runOnce(launch::stop, launch),                        // Stop launch wheels
                Commands.waitSeconds(0.2),                                     // Hold condition for 0.2 seconds
                Commands.parallel(
                  Commands.runOnce(() -> launch.setLaunchRPM(4000), launch),   // Set RPM PID to 4000 RPM
                  Commands.runOnce(tilt::feedExtend, tilt)                     // Feed note into launcher after 2.0 seconds of warm up
                          .beforeStarting(Commands.waitSeconds(2.0))),
                Commands.parallel(
                  Commands.runOnce(launch::stop, launch),                      // Turn everything off
                  Commands.runOnce(tilt::feedRetract, tilt))
            )));
  }

  /** DRIVE ACCROSS THE LINE **/
  public static Command driveLine(SwerveSubsystem drivebase) {
    return Commands.sequence(
      Commands.runOnce(drivebase::zeroGyro, drivebase),
      Commands.waitSeconds(2.0),
      new RunCommand(() -> drivebase.driveCommand(() -> 0.4, () -> 0.0, () -> 0.0), drivebase)
        .withTimeout(4.0),
      new RunCommand(
        () -> drivebase.driveCommand(() -> 0.0, () -> 0.0, () -> 0.0), drivebase), null)
    ;
  }

  // public Command driveFieldOrientedAnglularVelocity = SwerveSubsystem.driveCommand(
  //   () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.kDriverDb_LeftY),
  //   () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.kDriverDb_LeftX),
  //   () -> driverXbox.getRightX() * 0.75);
  // private Autos() {
  //   throw new UnsupportedOperationException("This is a utility class!");
  // }

  public Command none() {
    return Commands.none();
  }
}
