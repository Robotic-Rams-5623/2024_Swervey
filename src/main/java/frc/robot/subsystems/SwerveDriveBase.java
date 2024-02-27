// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveUtils;

/** 
 * Create a new swerve module for each of the four modules.
 * Follow the orientation described below in the handy ASCII
 * image:
 *                (Front)
 *            1,2        3,4
 *              ==========
 *              =        =
 *              =        =
 *              ==========
 *            7,8        5,6
 *                (Back)
 *  (1,2) First number corresponds to the turn motor controller
 * and the second number corresponds to the drive motor controller.
*/

public class SwerveDriveBase extends SubsystemBase {
  private boolean fieldOriented = false;
  private double keepAngle = 0.0;
  private double timeSinceRot = 0.0;
  private double lastRotTime = 0.0;
  private double timeSinceDrive = 0.0;
  private double lastDriveTime = 0.0;

  private final PIDController m_keepAnglePID = new PIDController(0.3, 0.0, 0.0);

  private final Timer m_keepAngleTimer = new Timer();

  private SlewRateLimiter m_slewX = new SlewRateLimiter(Constants.Swerve.kTransSlewRate);
  private SlewRateLimiter m_slewY = new SlewRateLimiter(Constants.Swerve.kTransSlewRate);
  private SlewRateLimiter m_slewRot = new SlewRateLimiter(Constants.Swerve.kRotSlewRate);

  /** 
   * Create the gyro sensor that will help with the odometry of the driving
   * so the robot knows what direction it is facing. We are using the new
   * NAVX2 micro navigation board. Currently it is connected to the roboRIO
   * using the USB.
   */
  private final AHRS m_gyro = new AHRS(edu.wpi.first.wpilibj.SerialPort.Port.kUSB1); // Could also be USB2???

  /** SWERVE MODULES */
  // private SwerveModule[] m_swerveModules; // OR \\\Below///
  private SwerveModule m_FLModule = new SwerveModule("Front Left",
                                                     Constants.MotorIDs.kFrontLeftDriveCANid, 
                                                     Constants.MotorIDs.kFrontLeftTurnCANid, 
                                                     Constants.MotorIDs.kFrontLeftCANcoderid, 
                                                     Constants.Swerve.kFrontLeftAngleOffset,
                                                     true,
                                                     false,
                                                     false,
                                                     false);
  private SwerveModule m_FRModule = new SwerveModule("Front Right",
                                                     Constants.MotorIDs.kFrontRightDriveCANid,
                                                     Constants.MotorIDs.kFrontRightTurnCANid,
                                                     Constants.MotorIDs.kFrontRightCANcoderid,
                                                     Constants.Swerve.kFrontRightAngleOffset,
                                                     false,
                                                     false,
                                                     false,
                                                     false);
  private SwerveModule m_RRModule = new SwerveModule("Rear Right",
                                                     Constants.MotorIDs.kBackRightDriveCANid,
                                                     Constants.MotorIDs.kBackRightTurnCANid,
                                                     Constants.MotorIDs.kBackRightCANcoderid,
                                                     Constants.Swerve.kBackRightAngleOffset,
                                                     false,
                                                     false,
                                                     false,
                                                     false);
  private SwerveModule m_RLModule = new SwerveModule("Rear Left",
                                                     Constants.MotorIDs.kBackLeftDriveCANid,
                                                     Constants.MotorIDs.kBackLeftTurnCANid,
                                                     Constants.MotorIDs.kBackLeftCANcoderid,
                                                     Constants.Swerve.kBackLeftAngleOffset,
                                                     true,
                                                     false,
                                                     false,
                                                     false);
  
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
                                                    Constants.Swerve.kDriveKinematics,
                                                    m_gyro.getRotation2d(),
                                                    getModulePositions());

  private final SwerveDriveOdometry m_autoOdometry = new SwerveDriveOdometry(
                                                    Constants.Swerve.kDriveKinematics,
                                                    m_gyro.getRotation2d(),
                                                    getModulePositions());

  private final double[] m_latestSlew = { 0.0, 0.0, 0.0 };

  private SwerveModuleState[] m_desStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));


  /** 
   * CONSTRUCTS A NEW SWERVE DRIVETRAIN AND RESETS GYRO AND KEEP ANGLE PARAMETERS
   */
  public SwerveDriveBase() {
    // m_swerveModules = new SwerveModule[] {
    //   new SwerveModule("Front Left",
    //     Constants.MotorIDs.kFrontLeftDriveCANid,
    //     Constants.MotorIDs.kFrontLeftTurnCANid,
    //     Constants.MotorIDs.kFrontLeftCANcoderid,
    //     Constants.Swerve.kFrontLeftAngleOffset,
    //     false,
    //     false,
    //     false,
    //     false),
    //   new SwerveModule("Front Right",
    //     Constants.MotorIDs.kFrontRightDriveCANid,
    //     Constants.MotorIDs.kFrontRightTurnCANid,
    //     Constants.MotorIDs.kFrontRightCANcoderid,
    //     Constants.Swerve.kFrontRightAngleOffset,
    //     false,
    //     false,
    //     false,
    //     false),
    //   new SwerveModule("Back Left",
    //     Constants.MotorIDs.kBackLeftDriveCANid,
    //     Constants.MotorIDs.kBackLeftTurnCANid,
    //     Constants.MotorIDs.kBackLeftCANcoderid,
    //     Constants.Swerve.kBackLeftAngleOffset,
    //     false,
    //     false,
    //     false,
    //     false),
    //   new SwerveModule("Back Right",
    //     Constants.MotorIDs.kBackRightDriveCANid,
    //     Constants.MotorIDs.kBackRightTurnCANid,
    //     Constants.MotorIDs.kBackRightCANcoderid,
    //     Constants.Swerve.kBackRightAngleOffset,
    //     false,
    //     false,
    //     false,
    //     false)
    // };
    // swerveOdometry = new SwerveDriveOdometry(
    //   Constants.Swerve.kDriveKinematics,
    //   getYaw(),
    //   new SwerveModulePosition[] {
    //     m_swerveModules[0].getPosition(),
    //     m_swerveModules[1].getPosition(),
    //     m_swerveModules[2].getPosition(),
    //     m_swerveModules[3].getPosition()
    //   });
    m_keepAngleTimer.reset();
    m_keepAngleTimer.start();
    m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), new Pose2d());
    m_gyro.reset();

    stop();

    // AUTOBUILDER REQUIREMENTS??? (FRC TEAM 1706 CODE)
  }


  
  /**
   * DRIVE ROBOT USING JOYSTICK CONTROLS
   * 
   * @param xSpeed                Speed of robot in X-Driection (Forward).
   * @param ySpeed                Speed of robot in Y-Direction (Strafe).
   * @param rot                   Angular rate of robot rotation (Turning).
   * @param fieldRelative (bool)  Whether or not provided speeds are relative to field or not.
   * @param keepAngle             Whether or not to maintain angle relative to field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean keepAngle) {
    xSpeed = m_slewX.calculate(xSpeed);
    ySpeed = m_slewY.calculate(ySpeed);
    rot = m_slewRot.calculate(rot);

    m_latestSlew[0] = xSpeed;
    m_latestSlew[1] = ySpeed;
    m_latestSlew[2] = rot;

    if (keepAngle) {
      rot = performKeepAngle(xSpeed, ySpeed, rot); // Calls the keep angle function to update the keep angle or rotate
    }

    if (Math.abs(rot) < 0.03) {
      rot = 0.0;
    }
    if (Math.abs(xSpeed) < 0.03) {
      xSpeed = 0.0;
    }
    if (Math.abs(ySpeed) < 0.03) {
      ySpeed = 0.0;
    }

    if (fieldRelative) {
      setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d()));
    } else {
      setModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }
  }
  
  /**
   * Periodic runs once every scheduler call. Great place to put logging/dashboard
   * functions and math that needs to be done frequently.
   */
  @Override
  public void periodic() {

    double xSpeed = getChassisSpeed().vxMetersPerSecond;
    double ySpeed = getChassisSpeed().vyMetersPerSecond;

    double speed = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);

    SmartDashboard.putNumber("Speed", speed);

    SmartDashboard.putNumber("Front Left Encoder", m_FLModule.getAbsEncoder());
    SmartDashboard.putNumber("Front Right Encoder", m_FRModule.getAbsEncoder());
    SmartDashboard.putNumber("Rear Left Encoder", m_RLModule.getAbsEncoder());
    SmartDashboard.putNumber("Rear Right Encoder", m_RRModule.getAbsEncoder());

    SmartDashboard.putNumber("Front Left Turn Encoder", m_FLModule.getTurnPosition());
    SmartDashboard.putNumber("Front Right Turn Encoder", m_FRModule.getTurnPosition());
    SmartDashboard.putNumber("Rear Left Turn Encoder", m_RLModule.getTurnPosition());
    SmartDashboard.putNumber("Rear Right Turn Encoder", m_RRModule.getTurnPosition());

    updateOdometry();

    getPose();
  }
  
  /**
   * Sets the swerve ModuleStates based on states.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kMaxDriveMeterPerSec);
    m_FLModule.setDesiredState(desiredStates[0]);
    m_FRModule.setDesiredState(desiredStates[1]);
    m_RLModule.setDesiredState(desiredStates[2]);
    m_RRModule.setDesiredState(desiredStates[3]);
  }

  /**
   * Sets the swerve ModuleStates based on chassis speed.
   *
   * @param desiredStates The desired ChassisSpeeds of the modules.
   */
  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(secondOrderKinematics(chassisSpeeds));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kMaxDriveMeterPerSec);
    m_desStates = desiredStates;
    m_FLModule.setDesiredState(desiredStates[0]);
    m_FRModule.setDesiredState(desiredStates[1]);
    m_RLModule.setDesiredState(desiredStates[2]);
    m_RRModule.setDesiredState(desiredStates[3]);
  }

  /**
   * Sets the swerve ModuleStates based on chassis speed.
   *
   * @param desiredStates The desired ChassisSpeeds of the modules.
   */
  public ChassisSpeeds secondOrderKinematics(ChassisSpeeds chassisSpeeds) {
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Translation2d rotAdj = translation.rotateBy(new Rotation2d(-Math.PI / 2.0)).times(chassisSpeeds.omegaRadiansPerSecond * Constants.Swerve.kRotTransFactor);

    translation = translation.plus(rotAdj);

    return new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Stops all the swerve module motors so that nothing is moving.
   */
  public void stop() {
    m_FLModule.stop();
    m_FRModule.stop();
    m_RLModule.stop();
    m_RRModule.stop();
  }

  /**
   * Changes the idle mode of the drive motors to either Brake (true) or Coasr (false)
   *
   * @param enabled Brake/Coast idle mode
   */
  public void brakeMode(boolean enabled) {
    m_FLModule.enableBrake(enabled);
    m_FRModule.enableBrake(enabled);
    m_RLModule.enableBrake(enabled);
    m_RRModule.enableBrake(enabled);
  }

  /**
   * Gets the tilt and tilt velocity of the gyro. This is not useful in the 2024 game
   * but could be useful for controlling the balance on something in the future.
   */
  public double getTilt() {
    return m_gyro.getRoll();
  }
  public double getTiltVel() {
    return m_gyro.getRawGyroY();
  }

  /**
   * Updates odometry (teleop & auto variations) for the swerve drivetrain. This should be called
   * once per loop to minimize error.
   */
  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), getModulePositions());
  }
  public void updateAutoOdometry() {
    m_autoOdometry.update(m_gyro.getRotation2d(), getModulePositions());
  }

  /**
   * Function to retrieve latest robot gyro angle.
   * 
   * @return Rotation2d object containing Gyro angle
   */
  public Rotation2d getGyro() {
    return m_gyro.getRotation2d();
  }

  /**
   * Function (teleop & auto variations) created to retreieve and push the robot pose to the SmartDashboard
   * for diagnostics
   * 
   * @return Pose2d object containing the X and Y position and the heading of the
   *         robot.
   */
  public Pose2d getPose() {
    Pose2d pose = m_odometry.getPoseMeters();
    Translation2d position = pose.getTranslation();

    SmartDashboard.putNumber("Robot X", position.getX());
    SmartDashboard.putNumber("Robot Y", position.getY());
    SmartDashboard.putNumber("Robot Gyro", getGyro().getRadians());

    return pose;
  }
  public Pose2d getAutoPose() {
    updateAutoOdometry();
    Pose2d pose = m_autoOdometry.getPoseMeters();
    Translation2d position = pose.getTranslation();
    return m_autoOdometry.getPoseMeters();
  }

  /**
   * Resets the odometry and gyro to the specified position.
   *
   * @param position in which to set the odometry and gyro.
   */
  public void resetOdometry(Pose2d pose) {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    updateKeepAngle();
    m_odometry.resetPosition(m_gyro.getRotation2d().times(-1.0), getModulePositions(), pose);
    m_autoOdometry.resetPosition(m_gyro.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  /**
   * Resets the gyro to the given angle
   * 
   * @param angle the angle of the robot to reset to
   */
  public void resetOdometry(Rotation2d angle) {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(angle.getDegrees());
    Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
    updateKeepAngle();
    m_odometry.resetPosition(m_gyro.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  /**
   * Sets the odometry of the robot to the specified position
   * 
   * @param position in which to set the odometry to.
   */
  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  /**
   * Converts the 4 swerve module states into a chassisSpeed by making use of the
   * swerve drive kinematics.
   * 
   * @return ChassisSpeeds object containing robot X, Y, and Angular velocity
   */
  public ChassisSpeeds getChassisSpeed() {
    return Constants.Swerve.kDriveKinematics.toChassisSpeeds(m_FLModule.getState(),
                                                            m_FRModule.getState(),
                                                            m_RLModule.getState(),
                                                            m_RRModule.getState());
  }
  public ChassisSpeeds getCorDesChassisSpeed() {
    return Constants.Swerve.kDriveKinematics.toChassisSpeeds(m_desStates[0],
                                                            m_desStates[1],
                                                            m_desStates[2],
                                                            m_desStates[3]);
  }
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_FLModule.getPosition(),
      m_FRModule.getPosition(),
      m_RLModule.getPosition(),
      m_RRModule.getPosition()
    };
  }

  /**
   * Keep angle function is performed to mitigate drivetrain drift without the need
   * of the driver to manually adjust all the time.
   * A PIDController is used to attempt to maintain the robot heading to the keepAngle
   * value. This value is updated when the robot is rotated manually by the driver input.
   * 
   * @return rotation command in radians/s
   * @param xSpeed is the input drive X speed command
   * @param ySpeed is the input drive Y speed command
   * @param rot    is the input drive rotation speed command
   */
  private double performKeepAngle(double xSpeed, double ySpeed, double rot) {
    double output = rot; // Output shouold be set to the input rot command unless the Keep Angle PID is
                         // called
    if (Math.abs(rot) >= 0.01) { // If the driver commands the robot to rotate set the
                                 // last rotate time to the current time
      lastRotTime = m_keepAngleTimer.get();
    }
    if (Math.abs(xSpeed) >= 0.01
        || Math.abs(ySpeed) >= 0.01) { // if driver commands robot to translate set the
                                       // last drive time to the current time
      lastDriveTime = m_keepAngleTimer.get();
    }
    timeSinceRot = m_keepAngleTimer.get() - lastRotTime; // update variable to the current time - the last rotate time
    timeSinceDrive = m_keepAngleTimer.get() - lastDriveTime; // update variable to the current time - the last drive
                                                             // time
    if (timeSinceRot < 0.25) { // Update keepAngle up until 0.5s after rotate command stops to allow rotation
                               // move to finish
      keepAngle = getGyro().getRadians();
    } else if (Math.abs(rot) <= 0.01 && timeSinceDrive < 0.25) { // Run Keep angle pid
                                                                 // until 0.75s after drive
                                                                 // command stops to combat
                                                                 // decel drift
      output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle); // Set output command to the result of the
                                                                            // Keep Angle PID
    }
    return output;
  }

  public void updateKeepAngle() {
    keepAngle = getGyro().getRadians();
  }

  public void changeSlewRate(double translation, double rotation) {
    m_slewX = new SlewRateLimiter(translation, -translation, m_latestSlew[0]);
    m_slewY = new SlewRateLimiter(translation, -translation, m_latestSlew[1]);
    m_slewRot = new SlewRateLimiter(rotation, -rotation, m_latestSlew[2]);
  }
}
