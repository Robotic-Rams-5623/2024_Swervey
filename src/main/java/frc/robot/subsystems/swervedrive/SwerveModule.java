package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule
{
  /* Swerve Module Configurations Options */
  public final  SwerveModuleConfig      configuration;
  
  /* Swerve Module Motors */
  public final  SwerveMotor             angleMotor, driveMotor;
  // public final CANSparkFlex             angleMotor, driveMotor;
  // public final RelativeEncoder          encoder;
  // public final SparkPIDController       pid;
  
  /* Swerve Module Absolute Encoder */
  public final  SwerveAbsoluteEncoder   absoluteEncoder;
  
  /* Module ID Number 0 to 3 (FrontLeft -> FrontRight -> BackLeft -> BackRight) */
  public        int                     moduleNumber;
  
  /* Feedforward for Drive Motor During Closed Loop Mode */
  public        SimpleMotorFeedforward  feedforward;
  
  /* Maximum Speed of drive motors in meters persecond */
  public        double                  maxSpeed;
  
  /* Last Swerve Module State Applied */
  private       SwerveModuleState       lastState;
  
  /* Offset for Absolute Encoder Angle in Degrees From Known Position */
  private       double                  angleOffset;
  
  /* Encoder Synchronization Queued */
  private       boolean                 synchronizeEncoderQueued = false;

  
  /**
   * Construct the swerve module and initialize the motors and encoder.
   *
   * @param moduleNumber          Module number for kinematics (0 through 3).
   * @param moduleConfiguration   Module constants containing CAN ID's and offsets.
   * @param driveFeedForward      Drive Motor Feedforward
   */
  public SwerveModule(int moduleNumber,
                     SwerveModuleConfig moduleConfiguration,
                     SimpleMotorFeedforward driveFeedforward)
  {
    this.moduleNumber = moduleNumber;
    configuration = moduleConfiguration;
    angleOffset = moduleConfiguration.angleOffset;

    // Initialize Feedforward for drive motor
    feedforward = driveFeedforward;

    // Create motors from configuration and reset them to defaults.
    angleMotor = moduleConfiguration.angleMotor;
    driveMotor = moduleConfiguration.driveMotor;
    angleMotor.factoryDefaults();
    driveMotor.factoryDefaults();

    // Configure voltage comp, current limit, and ramp rate.
    angleMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    driveMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    angleMotor.setCurrentLimit(configuration.physicalCharacteristics.angleMotorCurrentLimit);
    driveMotor.setCurrentLimit(configuration.physicalCharacteristics.driveMotorCurrentLimit);
    angleMotor.setLoopRampRate(configuration.physicalCharacteristics.angleMotorRampRate);
    driveMotor.setLoopRampRate(configuration.physicalCharacteristics.driveMotorRampRate);
  }
}
