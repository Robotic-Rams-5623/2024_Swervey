// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveUtils;

public class SwerveDriveBase extends SubsystemBase {
  private final StructArrayPublisher<SwerveModuleState> publisher;

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
  private final SwerveModuleBuilder m_frontLeft = new SwerveModuleBuilder(
    Constants.MotorIDs.kFrontLeftDriveCANid,
    Constants.MotorIDs.kFrontLeftTurnCANid,
    Constants.MotorIDs.kFrontLeftCANcoderid,
    Constants.Swerve.kFrontLeftAngleOffset,
    "Front Left"
  );

  private final SwerveModuleBuilder m_frontRight = new SwerveModuleBuilder(
    Constants.MotorIDs.kFrontRightDriveCANid,
    Constants.MotorIDs.kFrontRightTurnCANid,
    Constants.MotorIDs.kFrontRightCANcoderid,
    Constants.Swerve.kFrontRightAngleOffset,
    "Front Right"
  );

  private final SwerveModuleBuilder m_backLeft = new SwerveModuleBuilder(
    Constants.MotorIDs.kBackLeftDriveCANid,
    Constants.MotorIDs.kBackLeftTurnCANid,
    Constants.MotorIDs.kBackLeftCANcoderid,
    Constants.Swerve.kBackLeftAngleOffset,
    "Back Left"
  );

  private final SwerveModuleBuilder m_backRight = new SwerveModuleBuilder(
    Constants.MotorIDs.kBackRightDriveCANid,
    Constants.MotorIDs.kBackRightTurnCANid,
    Constants.MotorIDs.kBackRightCANcoderid,
    Constants.Swerve.kBackRightAngleOffset,
    "Back Right"
  );

  /** 
   * Create the gyro sensor that will help with the odometry of the driving
   * so the robot knows what direction it is facing. We are using the new
   * NAVX2 micro navigation board. Currently it is connected to the roboRIO
   * using the USB.
   */
  private final AHRS m_gyro = new AHRS(edu.wpi.first.wpilibj.SerialPort.Port.kUSB1); // Could also be USB2???

  /** 
   * These values are a Slew rate filter for controlling the lateral
   * acceleration.
   */
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.Swerve.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.Swerve.kRotationSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      Constants.Swerve.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  public SwerveDriveBase() {
    publisher = NetworkTableInstance.getDefault().
      getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).
      publish();
  }

  /**
   * PERIODIC LOOP 
   * Runs once a scheduler call
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left Angle", m_frontLeft.getAngle());
    SmartDashboard.putNumber("Front Right Angle", m_frontRight.getAngle());
    SmartDashboard.putNumber("Back Left Angle", m_backLeft.getAngle());
    SmartDashboard.putNumber("Back Right Angle", m_backRight.getAngle());
    /** 
     * Update the odemetry in the periodic block so it keeps getting updated
     */
    m_odometry.update(
      Rotation2d.fromDegrees(m_gyro.getYaw()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      });

      publisher.set(new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
      });
  }



  /**
   * Get the currently estimated position of the robot
   * 
   * @param 
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(m_gyro.getYaw()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      },
      pose);
  }

  public Command teleopDrive(
    DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation,
    BooleanSupplier fieldRelative, BooleanSupplier openLoop) {

      return run(() -> {
            double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.OperatorConstants.kDriverDeadband);
            double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.OperatorConstants.kDriverDeadband);
            double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.OperatorConstants.kDriverDeadband);

            
            
            boolean isOpenLoop = openLoop.getAsBoolean();

            translationVal *= Constants.Swerve.kMaxDriveMeterPerSec;

            strafeVal *= Constants.Swerve.kMaxDriveMeterPerSec;

            rotationVal *= Constants.Swerve.kMaxTurnRadianPerSec;
            drive(translationVal, strafeVal, rotationVal, fieldRelative.getAsBoolean(), isOpenLoop);
        }).withName("Teleop Drive");
  
  }
  
  /**
   * Standard drive control for the drive train. RobotContainer will use this to control
   * the robot with the controller.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
    ChassisSpeeds targetChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    setChassisSpeeds(targetChassisSpeeds, isOpenLoop, false);
  }

  public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace) {
    setModuleStates(Constants.Swerve.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds));//, openLoop, steerInPlace);
}

  /**
   * SET WHEELS TO X SHAPED CONFIGURATION
   * Sets the wheels so they are in an X position. This is great for locking up the drive
   * and making it very difficult to move for either defense or balancing on a platform.
   */
  public void setPositionX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * SET THE SWERVE MODULE STATES
   * Sets the swerve module states to whatever you give it
   * 
   * @param desiredStates = The desired state of the SwerveModules
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Swerve.kMaxDriveMeterPerSec);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * RESET THE DRIVE ENCODERS
   * Resets the drive encoders to zero. It will not reset the turn encoders because
   * those are in absolute mode and can not be reset.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** 
   * ZERO ROBOT HEADING
   * sets the heading of the robot to zero
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * GET ROBOT HEADING
   * gets the current robot heading in degrees
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  /**
   * GET THE RATE OF TURNING
   * THe rate in deg/sec that the robot is turning
   * 
   * @return turn rate of robot
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.Swerve.kGyroInversed ? -1.0 : 1.0);
  }
}
