package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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
    public static final double kDriverDb_LeftX = 0.1;
    public static final double kDriverDb_LeftY = 0.1;
    public static final double kDriverDb_RightX = 0.1;
    public static final double TURN_CONSTANT = 7.5
    ;
    
    public static final int kActionUSBPort = 1;
    public static final double kActionDeadband = 0.1;
    public static final double kActionDb_LeftY = 0.1;
  }

  public static class TiltAngles {
    public static final double kFloorAngle = 0.0; // Degrees
    public static final double kAmpAngle = 60.0; // Degrees
    public static final double kSpeakerAngle = 100.0; // Degrees
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
  
    /** NOTE HANDLER/TILTER MECHANISM IDs */
    public static final int kTiltMotorCANid = 34;

    public static final int kIntakeMotorCANid = 10;
  }


  public static class SwerveModule {
    // Distance between the centers of the left and right wheel in the robot
    public static final double kWheelBase = Units.inchesToMeters(22.0); // [Meter]
    // Distance between the centers of the front and back wheels on the robot
    public static final double kTrackWidth = Units.inchesToMeters(24.25); // [Meter]
    // Robot Mass (Weight - gravity)
    public static final double ROBOT_MASS = 43.0; // [Kg]
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.133; // 20ms loop time + .113 spark amx lag time
    
    // Wheel dimensions
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // [Meter]
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // [Meter]
  
    // Gear reduction ratio for the drive and turn motors
    public static final double kDriveMotorReduction = (6.75 / 1.0); // 6.75:1 SDS MK4 Module with L2 Gear Ratio
    public static final double kTurnMotorReduction = 12.8; // SDS MK4 Module Turn Gear Ratio
  
    // Max speed of the robot in feet per seconds
    public static final double kMaxDriveFeetPerSec = 9.5; // [Feet/Sec]
  }



  /**
   * CONSTATNS FOR USE WITH YAGSL
   */
  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.5, 0, 0);
  }
  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }



  public static class Climb {

  public static final int kProxLowerDIO = 1;
  public static final int kProxUpperDIO = 2;
  
    /** CLIMB SUBSYSTEM PARAMETERS */
  public static final double kSpeedUp = 0.9; // Percent Output
  public static final double kSpeedDown = 0.9; // Percent Output
  public static final double kChainHeight = 24.5;
  public static final double kSpeakerHeight = 10;

  // Bottom Hook to About Ground
  public static final double kResetPosition = 15.5; // Zero for relative to robot or offset to be relative to floor
  // Reset position is like 11 inches from the bottom of the frame to the center of the bottom hook.

  /** CLIMB MOTOR CONFIGURATION CONSTANTS */
  public static final double kClosedRampRate = .5;
  public static final double kOpenRampRate = 0.5;
  public static final int kCurrentLimit = 60;
  public static final boolean kMotorInverted = true;
  public static final IdleMode kIdleMode = IdleMode.kBrake;

  /** CLIMB ENCODER CONFIGURATION CONSTANTS */
  // public static final boolean kEncInverted = false;
  public static final double kEncVelConversion = 1; // Inch/Sec = RPM * ???
  // 9 rots of motor : 1 rot of screw. 1 rot  of screw : 1/8 inch height
  public static final double kEncPosConversion = 1; // 72 Motor rot per inch

  /** CLIMB PID CONTROLLER CONSTANTS */
  public static final double kP = 0.06; // Proportional Constant
  public static final double kI = 0.0; // Integral Constant
  public static final double kD = 0.0; // Derivative Constant (LEAVE AT ZERO)
  public static final double kIz = 0.0; // Integral deadzone (Zero disables the deadzone)
  public static final double kFF = 0.00; // Feed Forward Gain
  public static final double kMaxOutput = 0.7; // Percent Output
  public static final double kMinOutput = -0.6; // Needs to be negative
  public static final double kMaxRPM = 5600; // Max RPM in Closed Loop Mode
  }



  public static class Handler {
    /** INTAKE SUBSYSTEM PARAMETERS */
    public static final int kTiltPotAIport = 1;
  
    public static final double kTiltMaxRange = 180; // What is the angle at 5 volts of the pot
    public static final double kTiltZeroAngle = 0; // What is the angle at 0 volts of the pot
    public static final double kTiltMaxAngle = 175;
    public static final double kTiltMinAngle = 0;
    
    public static final double kAmpPosition = 90;
  
    public static final double kUpSpeed = 0.15;
    public static final double kDownSpeed = 0.2;
    public static final double kManualSpeedLimit = .8;
    
    // public static final double kSpeedIn = 0.3; // Percent Output
    // public static final double kSpeedOut = 0.6; // Percent Output (Spit out faster then you take in)
  
    /** TITLER MOTOR CONFIGURATION CONSTANTS */
    public static final double kClosedRampRate = 0.1;
    public static final double kOpenRampRate = 0.5;
    public static final int kCurrentLimit = 40;
    public static final boolean kMotorInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
  
    /** TILTER PID CONTROLLER CONSTANTS */
    public static final double kP = 0.01; // Proportional Constant
    public static final double kI = 0; // Integral Constant
    public static final double kD = 0; // Derivative Constant (LEAVE AT ZERO)
    public static final double kIz = 0; // Integral deadzone (Zero disables the deadzone)
    public static final double kFF = 0; // Feed Forward Gain
    public static final double kMaxOutput = 0.6; // Percent Output to Raise the Mechanism
    public static final double kMinOutput = -0.3; // Negative Percent Output to Lower Mechanism
    public static final double kMaxRPM = 3000; // Max RPM in Closed Loop Mode
  
    public static final double kStatic = 0.2; // Static Feedforward gain
    public static final double kVel = 0.02; // Velocity feedforward gain
    public static final double kMaxVelRadPerSec = Math.PI / 4; // Max velocity of movement profile in radians per second (~45 deg per sec)
    public static final double kMaxAccRadPerSec = kMaxVelRadPerSec / 60; // Max acceleration of movement profile in radians per second squared
    public static final double kStartAngleOffset = 0.0;

    public static enum handleState {
      None,
      High,
      Low,
      Intake,
      Middle
    }
  }


  
  public static class Launcher{
    // LAUNCHER MOTOR CONFIGURATION CONSTANTS
    public static final double kSpeedPushLowRPM = 2500;
    public static final double kSpeedPushHighRPM = 4500;
    public static final double kSpeedPushPercent = 0.75; //````````````````

    public static final double kSpeedPull = 0.1; //````````````````
    public static final double kSpeedPullFast = 0.8;
    
    public static final double kClosedRampRate = .2;
    public static final double kOpenRampRate = .2;
    public static final boolean kLeftMotorInverted = true;
    public static final boolean kRightMotorInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kCoast;
    public static final int kCurrentLimit = 40;
  
    public static final boolean kEncLeftInverted = false;
    public static final boolean kEncRightInverted = true;
    public static final double kEncVelConversion = 1; // RPM = RPM * 1
    public static final double kEncPosConversion = 1; // There is no position, it's a spinning wheel!
  
    /** LAUNCHER PID CONTROLLER CONSTANTS */
    public static final double kP = 0.5; // Proportional Constant
    public static final double kI = 0.0; // Integral Constant
    public static final double kD = 0.0; // Derivative Constant (LEAVE AT ZERO)
    public static final double kIz = 0.0; // Integral deadzone (Zero disables the deadzone)
    public static final double kFF = 0.0; // Feed Forward Gain
    public static final double kMaxOutput = 0.95; // Percent Output
    public static final double kMinOutput = -0.95; // Needs to be negative
    public static final double kMaxRPM = 5600; // Max RPM in Closed Loop Mode
    
    // NOTE HANDLING AND LAUNCHING STATES FOR *GLOBAL VARIABLE*
    // public static enum noteState {
    //   None,
    //   Loading,
    //   Loaded,
    //   FiringLow,
    //   FiringHigh,
    //   Override
    // }
  }



  public static class Intake{
    public static final int kIntakeProxDIport = 0;
    
    /** INTAKE SUBSYSTEM PARAMETERS */
    public static final double kSpeedIn = 0.1; // Percent Output
    public static final double kSpeedOut = 0.5; // Percent Output

    /** INTAKE MOTOR CONFIGURATION CONSTANTS */
    public static final double kClosedRampRate = 0.1;
    public static final double kOpenRampRate = 0.1;
    public static final int kCurrentLimit = 35;
    public static final boolean kMotorInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
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
