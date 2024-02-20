package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Handler extends SubsystemBase {
  /* Create the motor for tilting the handling mechanism */
  private CANSparkMax m_TiltMotor = new CANSparkMax(Constants.MotorIDs.kTiltMotorCANid, MotorType.kBrushless);

  /* Angle Sensor Object
   * THe angle of the tilting mechanish will be read from a potentiometer rather than
   * an encoder. The potentiometer won't need to be zeroed everytime and will provide
   * a more reliable reading of the angle since the WPILIB library for potentiometers
   * include setting the angle range of the input directly in the object creation.
   */
  AnalogInput m_PotAI;
  AnalogPotentiometer m_TiltAngle;
  
  /* Solenoid Object
   * Since the solenoid we are using to feed the note into the launch wheels is a 12V
   * solneoid and we don't want to over complicate the circuitry with a relay controlled
   * by the RoboRIO DIO ports, lets run the solenoid off of the switchable port on the PDH!
   * 
   * IMPORTANT: REV ROBOTICS POWER ISTRIBUTION HUB (PDH) NEEDS TO HAVE CAN ID OF 1
   */
  private PowerDistribution m_PDH = new PowerDistribution(1, ModuleType.kRev);
  
  /* Create the sensors used for note sensing */
  private DigitalInput m_NoteProx = new DigitalInput(Constants.Handler.kHandlerProxDIport);

  /* PID Controller */
  // THIS SUBSYSTEM IS GOING TO BE CONVERTED TO A PID SUBSYSTEM IN ORDER TO BETTER CONTROL
  // THE ANGLE OF THE TILTING MECHANISM IN A WAY THAT IS EASIEST WITH COMMANDS.
  // private PIDController
  

  public Handler() {
    /** 
     * Do all the configuration stuff for the above objects and the subsystem.
     */
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
    m_TiltMotor.setInverted(Constants.Handler.kLeftMotorInverted);
    m_TiltMotor.setSmartCurrentLimit(Constants.Handler.kCurrentLimit); // 30 Amp Limit (40 Amp Breaker)

    m_TiltMotor.burnFlash();
  }



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


  
  @Override
  public void periodic() {
    // Slap some smartdashboard stuff in here
  }
}
