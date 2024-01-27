// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverUSBPort = 0;
    public static final double kDriverDeadband = 0.05;
    public static final int kActionUSBPort = 1;
    public static final double kActionDeadband = 0.05;
  }

  public static class MotorIDs {
    /** SPARK FLEX CAN IDs */
    public static final int kFrontLeftDriveCANid = 11;
    public static final int kFrontLeftTurnCANid = 12;
    public static final int kFrontLeftCANcoderid = 22;
    public static final int kFrontRightDriveCANid = 13;
    public static final int kFrontRightTurnCANid = 14;
    public static final int kFrontRightCANcoderid = 24;
    public static final int kBackLeftDriveCANid = 15;
    public static final int kBackLeftTurnCANid = 16;
    public static final int kBackLeftCANcoderid = 26;
    public static final int kBackRightDriveCANid = 17;
    public static final int kBackRightTurnCANid = 18;
    public static final int kBackRightCANcoderid = 28;

  }

  public static class Swerve {
    /** DRIVE KINEMATIC CALCULATIONS */
    // The angular offsets of the modules relative to the chassis in radians
    /*
     *  {Need a pretty ascii image to describe angle offset directions}
     */
    public static final double kFrontLeftAngleOffset = -Math.PI / 2; // -pi/2 = -90deg
    public static final double kFrontRightAngleOffset = 0; // 0 = 0deg
    public static final double kBackLeftAngleOffset = Math.PI; // pi = 1800deg
    public static final double kBackRightAngleOffset = Math.PI / 2; // pi/2 = 90deg

    // Distance between the centers of the left and right wheel in the robot
    public static final double kWheelBase = Units.inchesToMeters(22.0);
    // Distance between the centers of the front and back wheels on the robot
    public static final double kTrackWidth = Units.inchesToMeters(24.25);
    // The combination of the above into the kinematic positions of the wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    /** SPEED LIMIT CONSTANTS */
    public static final double kMaxDriveMeterPerSec = 2.0; // 2.0 meters/sec = 6.56 ft/sec
    public static final double kMaxTurnRadianPerSec = 2 * Math.PI; // 2pi/sec = 360 deg/sec!

    /** SLEW RATE CONSTANT VALUES */
    public static final double kMagnitudeSlewRate = 0.0;
    public static final double kRotationSlewRate = 0.0;

    /** GYROMETER CONSTANTS */
    public static final boolean kGyroInversed = false;
  }

  public static class SwerveModule {
    /** TURN ENCODER INVERSION */
    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor
    public static final boolean kTurnEncoderInverted = true;

    /** POSITION AND VERLOCITY CONSTANTS AND MATH */
    // Wheel dimensions
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // meters
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // meters

    // Gear reduction ratio from drive motor to wheel
    public static final double kDriveMotorReduction = (6.75 / 1.0); // 6.75:1 SDS MK4 Module with L2 Ratio
    // Free speed limits of the drive motor
    public static final double kDriveMotorFreeSpeedRps = MotorParams.kFreeSpeedRPMVortex / 60; // rev/sec
    public static final double kDriveWheelFreeSpeedRps = (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDriveMotorReduction; // rev/sec
    // Drive encoder position/velocity conversion factors
    public static final double kDriveEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDriveMotorReduction; // meters
    public static final double kDriveEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDriveMotorReduction) / 60.0; // meters per second

    // Turn encoder position/velocity conversion factors
    public static final double kTurnEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    // Turn encoder PID input limits, basically just the limits of a circle (zero to 2 pi)
    public static final double kTurnEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurnEncoderPositionPIDMaxInput = kTurnEncoderPositionFactor; // radians

    /** DRIVE AND TURN PID GAINS AND CONSTANTS */
    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDriveMinOutput = -1;
    public static final double kDriveMaxOutput = 1;

    public static final double kTurnP = 1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnFF = 0;
    public static final double kTurnMinOutput = -1;
    public static final double kTurnMaxOutput = 1;

    /** ADDITIONAL MOTOR SETTINGS THAT ARE THE SAME TO EACH MODULE */
    // Brake mode for these motors or we will get funky control issues
    public static final IdleMode kDriveMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurnMotorIdleMode = IdleMode.kBrake;
    // Limit current to that of the brakers for now, maybe lower if we blow any
    public static final int kDriveMotorCurrentLimit = 40; // amps
    public static final int kTurnMotorCurrentLimit = 40; // amps
  }


  /**
   * Constant parameters that are specific to the motor that is being used
   * for the given location on the robot when using special odometry controls
   * like in swerve drive.
   */
  public static class MotorParams {
    public static final double kFreeSpeedRPMVortex = 6784;
    public static final double kFreeSpeedRPMNeo = 5676;
    public static final double kFreeSpeedRPM550 = 11000;
  }
}
