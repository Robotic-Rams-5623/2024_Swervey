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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  private SwerveModule[] m_swerveModules;
  private SwerveDriveOdometry swerveOdometry;
  private SwerveDriveKinematics swerveKinematics;
  
  private Field2d m_field;

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
  // private double m_currentRotation = 0.0;
  // private double m_currentTranslationDir = 0.0;
  // private double m_currentTranslationMag = 0.0;

  // private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.Swerve.kMagnitudeSlewRate);
  // private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.Swerve.kRotationSlewRate);
  // private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  public SwerveDriveBase() {
    publisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    
    m_swerveModules = new SwerveModule[] {
      new SwerveModule("Front Left",
        Constants.MotorIDs.kFrontLeftDriveCANid,
        Constants.MotorIDs.kFrontLeftTurnCANid,
        Constants.MotorIDs.kFrontLeftCANcoderid,
        Constants.Swerve.kFrontLeftAngleOffset,
        false,
        false,
        false,
        false),
      new SwerveModule("Front Right",
        Constants.MotorIDs.kFrontRightDriveCANid,
        Constants.MotorIDs.kFrontRightTurnCANid,
        Constants.MotorIDs.kFrontRightCANcoderid,
        Constants.Swerve.kFrontRightAngleOffset,
        false,
        false,
        false,
        false),
      new SwerveModule("Back Left",
        Constants.MotorIDs.kBackLeftDriveCANid,
        Constants.MotorIDs.kBackLeftTurnCANid,
        Constants.MotorIDs.kBackLeftCANcoderid,
        Constants.Swerve.kBackLeftAngleOffset,
        false,
        false,
        false,
        false),
      new SwerveModule("Back Right",
        Constants.MotorIDs.kBackRightDriveCANid,
        Constants.MotorIDs.kBackRightTurnCANid,
        Constants.MotorIDs.kBackRightCANcoderid,
        Constants.Swerve.kBackRightAngleOffset,
        false,
        false,
        false,
        false)
    };

    swerveOdometry = new SwerveDriveOdometry(
      Constants.Swerve.kDriveKinematics,
      getYaw(),
      new SwerveModulePosition[] {
        m_swerveModules[0].getPosition(),
        m_swerveModules[1].getPosition(),
        m_swerveModules[2].getPosition(),
        m_swerveModules[3].getPosition()
      });

      m_field = new Field2d();

      SmartDashboard.putData("Field", m_field);
  }

  /**
   * Get the angle heading of the robot about the z- axis.
   * 
   * @return Rotation2d of robot Yaw angle
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  /** 
   * ZERO ROBOT HEADING
   * sets the heading of the robot to zero
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * PERIODIC LOOP 
   * Runs once a scheduler call
   */
  @Override
  public void periodic() {
    /** 
     * Update the odemetry in the periodic block so it keeps getting updated
     */
    swerveOdometry.update(
      getYaw(),
      new SwerveModulePosition[] {
        m_swerveModules[0].getPosition(),
        m_swerveModules[1].getPosition(),
        m_swerveModules[2].getPosition(),
        m_swerveModules[3].getPosition()
      });

      m_field.setRobotPose(getPose());
      
      for (SwerveModule mod : m_swerveModules) {
        SmartDashboard.putNumber(
          mod.modName + " CANcoder", mod.getCANcoderAngle());
        SmartDashboard.putNumber(
          mod.modName + " Turn", mod.getWheelAngle().getDegrees());
        SmartDashboard.putNumber(
          mod.modName + " Position", mod.getDriveEncoderPosition());
        SmartDashboard.putNumber(
          mod.modName + " Drive Velocity", mod.getDriveVelocity());
      }
  }

  /**
   * Get the currently estimated position of the robot.
   * 
   * @return Pose2d position of the robot
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Reset the currently estimated position of the robot.
   * 
   * @param pose
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(
      getYaw(),
      new SwerveModulePosition[] {
        m_swerveModules[0].getPosition(),
        m_swerveModules[1].getPosition(),
        m_swerveModules[2].getPosition(),
        m_swerveModules[3].getPosition()
      }, pose
    );
  }

  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      m_swerveModules[0].getState(),
      m_swerveModules[1].getState(),
      m_swerveModules[2].getState(),
      m_swerveModules[3].getState()
    };
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
    
    m_swerveModules[0].setState(desiredStates[0], false, true);
    m_swerveModules[1].setState(desiredStates[1], false, true);
    m_swerveModules[2].setState(desiredStates[2], false, true);
    m_swerveModules[3].setState(desiredStates[3], false, true);
  }

  public void drive(
    Translation2d translation,
    double rotation,
    boolean fieldRelative,
    boolean isOpenLoop) {
      
      SwerveModuleState[] m_SwerveModuleStates = 
        Constants.Swerve.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(
                  translation.getX(), translation.getY(), rotation));
      
      SwerveDriveKinematics.desaturateWheelSpeeds(m_SwerveModuleStates, Constants.Swerve.kMaxDriveMeterPerSec);

      m_swerveModules[0].setState(m_SwerveModuleStates[0], isOpenLoop, true);
      m_swerveModules[1].setState(m_SwerveModuleStates[1], isOpenLoop, true);
      m_swerveModules[2].setState(m_SwerveModuleStates[2], isOpenLoop, true);
      m_swerveModules[3].setState(m_SwerveModuleStates[3], isOpenLoop, true);
      
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
