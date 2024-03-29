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
  public static final double kDriverDeadband = 0.1;
  public static final double kCubic = 0.95;
  public static final double kLinear = 0.05;
  public static final int kActionUSBPort = 1;
  public static final double kActionDeadband = 0.1;
}


  
public static class MotorIDs {
  /** SPARK FLEX CAN IDs for SWERVE DRIVE*/
  public static final int kFrontLeftDriveCANid = 17;
  public static final int kFrontLeftTurnCANid = 18;
  public static final int kFrontLeftCANcoderid = 22;
  public static final int kFrontRightDriveCANid = 15;
  public static final int kFrontRightTurnCANid = 16;
  public static final int kFrontRightCANcoderid = 24;
  public static final int kBackLeftDriveCANid = 11;
  public static final int kBackLeftTurnCANid = 12;
  public static final int kBackLeftCANcoderid = 26;
  public static final int kBackRightDriveCANid = 13;
  public static final int kBackRightTurnCANid = 14;
  public static final int kBackRightCANcoderid = 28;

  /** SPARK MAX CLIMB IDs */
  public static final int kClimbMotorCANid = 31;

  /** LAUNCHER IDs */
  public static final int kRLauncherMotorCANid = 32; 
  public static final int kLLauncherMotorCANid = 33;
  public static final int kLaunchFeedServoid = 0;

  /** INTAKE IDs */
  public static final int kIntakeMotorCANid = 10;

  /** NOTE HANDLER/TILTER MECHANISM IDs */
  public static final int kTiltMotorCANid = 34;
}


  
public static class Handler {
  /** INTAKE SUBSYSTEM PARAMETERS */
  public static final int kHandlerProxDIport = 0;
  public static final int kTiltPotAIport = 0;

  public static final double kTiltMaxRange = 100; // What is the angle at 5 volts of the pot
  public static final double kTiltZeroAngle = 100; // What is the angle at 0 volts of the pot

  public static final double kUpSpeed = 0.4;
  public static final double kDownSpeed = 0.3;
  
  // public static final double kSpeedIn = 0.3; // Percent Output
  // public static final double kSpeedOut = 0.6; // Percent Output (Spit out faster then you take in)

  /** TITLER MOTOR CONFIGURATION CONSTANTS */
  public static final double kClosedRampRate = 0.75;
  public static final double kOpenRampRate = 0.75;
  public static final int kCurrentLimit = 30;
  public static final boolean kMotorInverted = false;
  public static final IdleMode kIdleMode = IdleMode.kBrake;

  /** TILTER PID CONTROLLER CONSTANTS */
  public static final double kP = 0.01; // Proportional Constant
  public static final double kI = 0; // Integral Constant
  public static final double kD = 0; // Derivative Constant (LEAVE AT ZERO)
  public static final double kIz = 0; // Integral deadzone (Zero disables the deadzone)
  public static final double kFF = 0.001; // Feed Forward Gain
  public static final double kMaxOutput = 0.6; // Percent Output to Raise the Mechanism
  public static final double kMinOutput = -0.3; // Negative Percent Output to Lower Mechanism
  public static final double kMaxRPM = 3000; // Max RPM in Closed Loop Mode

  public static final double kStatic = 0.1; // Static Feedforward gain
  public static final double kVel = 0.01; // Velocity feedforward gain
}


  
public static class Climb{
  /** CLIMB SUBSYSTEM PARAMETERS */
  public static final double kSpeedUp = 0.6; // Percent Output
  public static final double kSpeedDown = 0.4; // Percent Output
  public static final double kResetPosition =11; // Zero for relative to robot or offset to be relative to floor
  // Reset position is like 11 inches from the bottom of the frame to the center of the bottom hook.

  /** CLIMB MOTOR CONFIGURATION CONSTANTS */
  public static final double kClosedRampRate = 2;
  public static final double kOpenRampRate = 2;
  public static final int kCurrentLimit = 30;
  public static final boolean kMotorInverted = true;
  public static final IdleMode kIdleMode = IdleMode.kBrake;

  /** CLIMB ENCODER CONFIGURATION CONSTANTS */
  public static final boolean kEncInverted = false;
  public static final double kEncVelConversion = 1; // Inch/Sec = RPM * ???
  public static final double kEncPosConversion = 1; // Inches = Units * ???

  /** CLIMB PID CONTROLLER CONSTANTS */
  public static final double kP = 0.1; // Proportional Constant
  public static final double kI = 0.0; // Integral Constant
  public static final double kD = 0.0; // Derivative Constant (LEAVE AT ZERO)
  public static final double kIz = 0.5; // Integral deadzone (Zero disables the deadzone)
  public static final double kFF = 0.01; // Feed Forward Gain
  public static final double kMaxOutput = 0.6; // Percent Output
  public static final double kMinOutput = -0.4; // Needs to be negative
  public static final double kMaxRPM = 5600; // Max RPM in Closed Loop Mode
}


  
public static class Launcher{
  // LAUNCHER MOTOR CONFIGURATION CONSTANTS
  public static final double kSpeedPushLowRPM = 2500;
  public static final double kSpeedPushHighRPM = 4000;
  public static final double kSpeedPull = 0.5;
  public static final double kSpeedPullFast = 0.8;
  
  public static final double kClosedRampRate = .2;
  public static final double kOpenRampRate = .2;
  public static final boolean kLeftMotorInverted = false;
  public static final boolean kRightMotorInverted = true;
  public static final IdleMode kIdleMode = IdleMode.kCoast;
  public static final int kCurrentLimit = 30;

  public static final boolean kEncLeftInverted = false;
  public static final boolean kEncRightInverted = true;
  public static final double kEncVelConversion = 1; // RPM = RPM * 1
  public static final double kEncPosConversion = 1; // There is no position, it's a spinning wheel

  /** LAUNCHER PID CONTROLLER CONSTANTS */
  public static final double kP = 0.001; // Proportional Constant
  public static final double kI = 0.0; // Integral Constant
  public static final double kD = 0.0; // Derivative Constant (LEAVE AT ZERO)
  public static final double kIz = 1.0; // Integral deadzone (Zero disables the deadzone)
  public static final double kFF = 0.001; // Feed Forward Gain
  public static final double kMaxOutput = 0.99; // Percent Output
  public static final double kMinOutput = -0.99; // Needs to be negative
  public static final double kMaxRPM = 5600; // Max RPM in Closed Loop Mode
  
  // NOTE HANDLING AND LAUNCHING STATES FOR *GLOBAL VARIABLE*
  public static enum noteState {
    None,
    Loading,
    Loaded,
    FiringLow,
    FiringHigh,
    Override
  }
}


  
public static class Swerve {
  /** DRIVE KINEMATIC CALCULATIONS */
  // The angular offsets of the modules relative to the chassis in radians
  /*
   *  {Need a pretty ascii image to describe angle offset directions}
   */

  // Set up robot so that the wheel bevel gears are all facing left and are straight as possible!
  // Replace the 0.0 inside the -Math.toRadians() with the recorded value.
  // Wheels spinning in the incorrect direction should get a +180deg (or PI) added to the offset.
  public static final double kFrontLeftAngleOffset = -6.191253662109375 + 6.596038818359375; 
  public static final double kFrontRightAngleOffset = -0.004592895507812 + 0.630447387695312; 
  public static final double kBackLeftAngleOffset = 0.0;
  public static final double kBackRightAngleOffset = 0.007659912109375 - 0.507720947265625; 
  
  // Distance between the centers of the left and right wheel in the robot
  public static final double kWheelBase = Units.inchesToMeters(22.0);
  // Distance between the centers of the front and back wheels on the robot
  public static final double kTrackWidth = Units.inchesToMeters(24.25);
  // The combination of the above into the kinematic positions of the wheels
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // RL
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // RR

  /** SPEED LIMIT CONSTANTS */
  public static final double kMaxDriveMeterPerSec = 2.0; // 2.0 meters/sec = 6.56 ft/sec
  public static final double kMaxTurnRadianPerSec = Math.PI * 2; // 2pi/sec = 360 deg/sec!
  public static final double kMaxTurnAccelerationRadiansPerSecSquared = Math.PI * 1.5;
  public static final double kRotTransFactor = 0.045;
  
  /** SLEW RATE CONSTANT VALUES */
  public static final double kTransSlewRate = 14.0;
  public static final double kRotSlewRate = 16.0;
  
  /** GYROMETER CONSTANTS */
  public static final boolean kGyroInversed = false;
}


  
public static class SwerveModule {
  /** TURN ENCODER INVERSION */
  // Invert the turning encoder, since the output shaft rotates in the opposite direction of
  // the steering motor
  public static final boolean kTurnEncoderInverted = false;

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
  public static final double kDriveEncoderPositionFactor = (1.0 / kDriveMotorReduction) * kWheelCircumferenceMeters; // meters
  public static final double kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0; // meters per second

  // Turn encoder position/velocity conversion factors
  public static final double kTurnEncoderPositionFactor = 1.0 / (kDriveMotorReduction) * 2 * Math.PI; // radians
  public static final double kTurnEncoderVelocityFactor = kTurnEncoderPositionFactor / 60.0; // radians per second
  // Turn encoder PID input limits, basically just the limits of a circle (zero to 2 pi)
  public static final double kTurnEncoderPositionPIDMinInput = 0; // radians  // NOT IMPORTANT
  public static final double kTurnEncoderPositionPIDMaxInput = kTurnEncoderPositionFactor; // radians  // NOT IMPORTANT

  /** DRIVE AND TURN PID GAINS AND CONSTANTS */
  public static final double kDriveP = 0.15;
  public static final double kDriveI = 0; // NOT IMPORTANT
  public static final double kDriveD = 0; // NOT IMPORTANT
  public static final double kDriveFF = 1 / kDriveWheelFreeSpeedRps; // NOT IMPORTANT
  public static final double kDriveMinOutput = -0.9; // NOT IMPORTANT
  public static final double kDriveMaxOutput = 0.9; // NOT IMPORTANT
  public static final double kDriveS = 0.01;
  public static final double kDriveV = 1.0 / (MotorParams.kFreeSpeedRPMVortex * kDriveEncoderVelocityFactor);

  public static final double kTurnP = 3.0;
  public static final double kTurnI = 0; // NOT IMPORTANT
  public static final double kTurnD = 0; // NOT IMPORTANT
  public static final double kTurnFF = 0; // NOT IMPORTANT
  public static final double kTurnMinOutput = -0.8; // NOT IMPORTANT
  public static final double kTurnMaxOutput = 0.8; // NOT IMPORTANT

  /** ADDITIONAL MOTOR SETTINGS THAT ARE THE SAME TO EACH MODULE */
  // Brake mode for these motors or we will get funky control issues
  public static final IdleMode kDriveMotorIdleMode = IdleMode.kBrake;
  public static final IdleMode kTurnMotorIdleMode = IdleMode.kBrake;
  // Limit current to that of the brakers for now, maybe lower if we blow any
  public static final int kDriveMotorCurrentLimit = 40; // amps
  public static final int kTurnMotorCurrentLimit = 20; // amps

  public static final double kSpeedDeadband = 0.05; // NOT IMPORTANT
  }


  
  /**
   * Constant parameters that are specific to the motor that is being used
   * for the given location on the robot when using special odometry controls
   * like in swerve drive.
   */
  public static class MotorParams {
    public static final double kFreeSpeedRPMVortex = 6700;//6784;
    public static final double kFreeSpeedRPMNeo = 5676;
    public static final double kFreeSpeedRPM550 = 11000;
  }
}
