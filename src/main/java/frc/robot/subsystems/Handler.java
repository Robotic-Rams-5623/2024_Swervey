// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Handler extends PIDSubsystem {
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

  /* Solenoid Object
   * Since the solenoid we are using to feed the note into the launch wheels is a 12V
   * solneoid and we don't want to over complicate the circuitry with a relay controlled
   * by the RoboRIO DIO ports, lets run the solenoid off of the switchable port on the PDH!
   * 
   * IMPORTANT: REV ROBOTICS POWER ISTRIBUTION HUB (PDH) NEEDS TO HAVE CAN ID OF 1
   */
  private final PowerDistribution m_PDH = new PowerDistribution(1, ModuleType.kRev);

  /* Create the sensors used for note sensing */
  private final DigitalInput m_NoteProx = new DigitalInput(Constants.Handler.kHandlerProxDIport);

  /* Simple motor feed forward controll to help counteract gravity */
  private final SimpleMotorFeedforward m_tiltFeedForward = new SimpleMotorFeedforward(Constants.Handler.kStatic, Constants.Handler.kVel);


  public Handler() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.Handler.kP, Constants.Handler.kI, Constants.Handler.kD));
    getController().setTolerance(5.0); // In Degrees

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

    m_TiltMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
    m_TiltMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
    m_TiltMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
    m_TiltMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 0);
    m_TiltMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 0);       
    m_TiltMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 0);       
    m_TiltMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 0); 

    m_TiltMotor.burnFlash();
  }

  /** 
   * NEXT FEW FUNCTIONS ARE FOR THE PID SUBSYSTEM
   */
  @Override
  public void useOutput(double output, double setpoint) {
    m_TiltMotor.set(output);
    // May need to add a feedforward to counteract gravity
  }

  @Override
  public double getMeasurement() {
    return getPotAngle();
  }



  /**
   * REMAINING FUNCTIONS THAT ARENT DIRECTLY ASSOCIATED WITH THE PID
   */

  /**
   *  ENERGIZE THE FEED SOLENOID
   * Energizing the feed solenoid by enabling the switchable port on the PDH will
   * push the note into the feed wheels and fire the note into the desired target.
   */
  public void feedExtend() {
    m_PDH.setSwitchableChannel(true);
  }

  /**
   * DE_ENERGIZE THE FEED SOLENOID
   * De-energizing the feed solenoid by disabling the switchable port on the PDH
   * will retract the note pusher back to its loading state.
   */
  public void feedRetract() {
    m_PDH.setSwitchableChannel(false);
  }

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

  /**
   * MANUALLY MOVE THE TILT UPWARDS
   * 
   */
  public void manualUp() {
    m_TiltMotor.set(Constants.Handler.kUpSpeed);
  }

  /**
   * MANUALLY MOVE THE TILT DOWNWARDS
   * 
   */
  public void manualDown() {
    m_TiltMotor.set(-Constants.Handler.kDownSpeed); // Needs to be negative
  }

  /**
   * STOP ALL MOTOR OUTPUT
   * 
   */
  public void stop() {
    m_TiltMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // Slap some smartdashboard stuff in here
  }
}
