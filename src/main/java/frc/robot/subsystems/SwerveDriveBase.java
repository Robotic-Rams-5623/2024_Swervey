// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveUtils;

public class SwerveDriveBase extends SubsystemBase {
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
  private final SwerveModule m_frontLeft = new SwerveModule(
    Constants.MotorIDs.kFrontLeftDriveCANid,
    Constants.MotorIDs.kFrontLeftTurnCANid,
    Constants.Swerve.kFrontLeftAngleOffset
  );

  private final SwerveModule m_frontRight = new SwerveModule(
    Constants.MotorIDs.kFrontRightDriveCANid,
    Constants.MotorIDs.kFrontRightTurnCANid,
    Constants.Swerve.kFrontRightAngleOffset
  );

  private final SwerveModule m_backLeft = new SwerveModule(
    Constants.MotorIDs.kBackLeftDriveCANid,
    Constants.MotorIDs.kBackLeftTurnCANid,
    Constants.Swerve.kBackLeftAngleOffset
  );

  private final SwerveModule m_backRight = new SwerveModule(
    Constants.MotorIDs.kBackRightDriveCANid,
    Constants.MotorIDs.kBackRightTurnCANid,
    Constants.Swerve.kBackRightAngleOffset
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


  public SwerveDriveBase() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

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

  /**
   * Standard drive control for the drive train. RobotContainer will use this to control
   * the robot with the controller.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.Swerve.kMagnitudeSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.Swerve.kMaxDriveMeterPerSec;
    double ySpeedDelivered = ySpeedCommanded * Constants.Swerve.kMaxDriveMeterPerSec;
    double rotDelivered = m_currentRotation * Constants.Swerve.kMaxTurnRadianPerSec;

    var swerveModuleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.Swerve.kMaxDriveMeterPerSec);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
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
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw()).getDegrees();
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
