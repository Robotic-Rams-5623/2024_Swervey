package swervelib.parser.json;

import static swervelib.telemetry.SwerveDriveTelemetry.canIdWarning;
import static swervelib.telemetry.SwerveDriveTelemetry.i2cLockupWarning;
import static swervelib.telemetry.SwerveDriveTelemetry.serialCommsIssueWarning;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.imu.ADXRS450Swerve;
import swervelib.imu.NavXSwerve;
import swervelib.imu.SwerveIMU;
import swervelib.motors.SparkFlexSwerve;
import swervelib.motors.SwerveMotor;

/**
 * Device JSON parsed class. Used to access the JSON data.
 */
public class DeviceJson
{

  /**
   * The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx
   */
  public String type;
  /**
   * The CAN ID or pin ID of the device.
   */
  public int    id;
  /**
   * The CAN bus name which the device resides on if using CAN.
   */
  public String canbus = "";

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the current configuration.
   *
   * @param motor {@link SwerveMotor} of which attached encoders will be created from, only used when the type is
   *              "attached" or "canandencoder".
   * @return {@link SwerveAbsoluteEncoder} given.
   */
  public SwerveAbsoluteEncoder createEncoder(SwerveMotor motor)
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "none":
        return null;
      case "cancoder":
        return new CANCoderSwerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveIMU} from the given configuration.
   *
   * @return {@link SwerveIMU} given.
   */
  public SwerveIMU createIMU()
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "adxrs450":
        return new ADXRS450Swerve();
      case "navx":
      case "navx_spi":
        return new NavXSwerve(SPI.Port.kMXP);
      case "navx_i2c":
        DriverStation.reportWarning(
            "WARNING: There exists an I2C lockup issue on the roboRIO that could occur, more information here: " +
            "\nhttps://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues" +
            ".html#onboard-i2c-causing-system-lockups",
            false);
        i2cLockupWarning.set(true);
        return new NavXSwerve(I2C.Port.kMXP);
      case "navx_usb":
        DriverStation.reportWarning("WARNING: There is issues when using USB camera's and the NavX like this!\n" +
                                    "https://pdocs.kauailabs.com/navx-mxp/guidance/selecting-an-interface/", false);
        serialCommsIssueWarning.set(true);
        return new NavXSwerve(Port.kUSB);
      case "navx_mxp_serial":
        serialCommsIssueWarning.set(true);
        return new NavXSwerve(Port.kMXP);
      default:
        throw new RuntimeException(type + " is not a recognized imu/gyroscope type.");
    }
  }

  /**
   * Create a {@link SwerveMotor} from the given configuration.
   *
   * @param isDriveMotor If the motor being generated is a drive motor.
   * @return {@link SwerveMotor} given.
   */
  public SwerveMotor createMotor(boolean isDriveMotor)
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "sparkflex":
        return new SparkFlexSwerve(id, isDriveMotor);
      default:
        throw new RuntimeException(type + " is not a recognized motor type.");
    }
  }
}
