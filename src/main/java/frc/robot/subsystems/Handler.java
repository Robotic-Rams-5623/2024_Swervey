package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfilledPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

public class Handler extends ProfiledPIDSubsystem {
  /* Create the motor for tilting the handling mechanism */
  private final CANSparkMax m_TiltMotor = new CANSparkMax(Constants.MotorIDs.kTiltMotorCANid, MotorType.kBrushless);

  /* Angle Sensor Object
   * THe angle of the tilting mechanish will be read from a potentiometer rather than
   * an encoder. The potentiometer won't need to be zeroed everytime and will provide
   * a more reliable reading of the angle since the WPILIB library for potentiometers
   * include setting the angle range of the input directly in the object creation.
   */
  private final AnalogInput m_PotAI;
  private final AnalogPotentiometer m_TiltAngle;

  /* Simple motor feed forward controll to help counteract gravity */
  private final SimpleMotorFeedforward m_tiltFeedForward = new SimpleMotorFeedforward(Constants.Handler.kStatic, Constants.Handler.kVel);

  public Handler() {
    super(
      // The PIDController used by the subsystem
      new ProfiledPIDController(
          Constants.Handler.kP, 
          Constants.Handler.kI, 
          Constants.Handler.kD, 
          new TrapezoidProfile.Constraints(
              Constants.Handler.kMaxVelRadPerSec,  // Max Velocity Radians Per Second (~45 degrees per second)
              Constants.Handler.kMaxAccRadPerSec   // Max Acceleration Rad Per Sec Squared
          )),
      0);

    // Start handler at rest in starting position
    setGoal(Constants.Handler.kStartAngleOffset);
    
    /* Configure the potentiometer for angle measurement of the tilt mechanism */
    m_PotAI = new AnalogInput(Constants.Handler.kTiltPotAIport);
    m_PotAI.setAverageBits(2);

    /* Using the assigned analog input object, the potentiometer is initalized. The library
     * handles assigning the port and converting the seen voltages to meaningful external units.
     * THe first parameter is the analog input object.
     * The second parameter is the value (in degrees) at its full range of motion.
     * The third parameter is the value (in degrees) where the mechanism is starting from at Zero Volts
     *      i.e. At zero Volts the tilter is at 30 degrees.
     */
    m_TiltAngle = new AnalogPotentiometer(m_PotAI, Constants.Handler.kTiltMaxRange, Constants.Handler.kTiltZeroAngle);

    /* Tilter Motor Configuration */
    m_TiltMotor.restoreFactoryDefaults();
    m_TiltMotor.setOpenLoopRampRate(Constants.Handler.kOpenRampRate); // Zero to Full Throttle
    m_TiltMotor.setClosedLoopRampRate(Constants.Handler.kClosedRampRate); // Zero to Full Throttle
    m_TiltMotor.setIdleMode(Constants.Handler.kIdleMode); // Brake or Coast
    m_TiltMotor.setInverted(Constants.Handler.kMotorInverted);
    m_TiltMotor.setSmartCurrentLimit(Constants.Handler.kCurrentLimit); // 30 Amp Limit (40 Amp Breaker)
    configureCANStatusFrames(m_TiltMotor, 100, 20, 20, 0, 0, 0, 0);
    m_TiltMotor.burnFlash();
  }

  public void configureCANStatusFrames(
      CANSparkMax motor, int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4, int CANStatus5, int CANStatus6)
  {
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, CANStatus5);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, CANStatus6);
  }

  /** 
   * NEXT FEW FUNCTIONS ARE FOR THE PID SUBSYSTEM
   */
  @Override
  public void useOutput(double output, double setpoint) {
    m_TiltMotor.set(
          output +
          m_tiltFeedForward.calculate(setpoin.position, setpoint.velocity)
    );
  }

  @Override
  public double getMeasurement() {
    return getPotAngle();
  }


  /**
   * REMAINING FUNCTIONS THAT ARENT DIRECTLY ASSOCIATED WITH THE PID
   */

  /**
   * GET THE ANALOG INPUT POTENTIOMETER VOLTAGE
   * @return voltage of potentiometer in volts
   */
  public double getPotVoltage() {
    return m_PotAI.getVoltage();
  }

  /**
   * GET THE ANALOG INPUT POTENTIOMETER ANGLE
   * @return angle in degrees of tilt mechanism
   */
  public double getPotAngle() {
    return m_TiltAngle.get();
  }

  public boolean atHighRange() {
    return (getPotAngle() >= (Constants.Handler.kTiltMaxAngle - 10.0));
  }

  public boolean atLowRange() {
    return (getPotAngle() <= (Constants.Handler.kTiltMinAngle + 5.0));
  }

  
  /**
   * MANUALLY MOVE THE TILT UPWARDS
   */
  public void manualUp() {
    double speed = 0;
    if (atHighRange()) {
      speed = 0.0;
    } else {
      speed = Constants.Handler.kUpSpeed;
    }
    m_TiltMotor.set(speed);
  }

  /**
   * MANUALLY MOVE THE TILT DOWNWARDS
   */
  public void manualDown() {
    double speed = 0;
    if (atLowRange()) {
      speed = 0.0;
    } else {
      speed = Constants.Handler.kDownSpeed;
    }
    m_TiltMotor.set(-speed);
  }

  /**
   * MANUAL VARIED SPEED
   */
  public Command manualTilt(DoubleSupplier speed) {
    return run(() -> {
      double input = MathUtil.applyDeadband(speed.getAsDouble() * Constants.Handler.kManualSpeedLimit, OperatorConstants.kDriverDb_LeftY);
      double angle = (getPotAngle()*90/65)+90;
      input += (.42 * Math.cos(Math.toRadians(angle)));
      m_TiltMotor.set(-Math.pow(input, 3) * Constants.Handler.kManualSpeedLimit);
      SmartDashboard.putNumber("Tilt Applied Input", input);
      SmartDashboard.putNumber("Tilt Applied Angle", angle);
    });
  }

  /**
   * STOP ALL MOTOR OUTPUT
   */
  public void stop() {
    m_TiltMotor.set(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pot Raw Voltage", getPotVoltage());
    SmartDashboard.putNumber("Pot Angle (Deg)", getPotAngle());
    SmartDashboard.putBoolean("Tilt Up Limit", atHighRange());
    SmartDashboard.putBoolean("Tilt Down Limit", atLowRange());
    SmartDashboard.putBoolean("Tilt PID Enabled?", isEnabled());
    SmartDashboard.putBoolean("Tilt PID At Setpoint?", atSetpoint());
    // SmartDashboard.putNumber("Tilt Current", m_TiltMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Tilt Applied Voltage", m_TiltMotor.getAppliedOutput());
  }
}
