// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  private CANSparkMax m_LauncherMotorLeft = new CANSparkMax(Constants.MotorIDs.kLLauncherMotorCANid, MotorType.kBrushless);
  private CANSparkMax m_LauncherMotorRight = new CANSparkMax(Constants.MotorIDs.kRLauncherMotorCANid, MotorType.kBrushless);

  private RelativeEncoder m_EncoderLeft;
  private RelativeEncoder m_EncoderRight;

  // private PIDController m_LauncherPIDController;
  private SparkPIDController m_LauncherPIDController;
  private double kP, kI, kD, kIz, kFF;

  /* The servo is really just a electro actuator. Need to find a way to give it 12V
   * Most likely from the switchable port on the PDH using a 5 to 10 amp fuse. */
  // private final Servo m_LauncherServo = new Servo(Constants.Launcher.kLauncherServoid);
  // private PowerDistribution m_powerHub = new PowerDistribution(0, ModuleType.kRev); // PROBABLY PUT THIS IN THE hANDLER SUBSYSTEM

  public Launcher() {
    /*
    *     LAUNCHER MOTORS CONFIGURATION
    * All the following configuration parameters can be set in the Constants file.
    */
    m_LauncherMotorLeft.restoreFactoryDefaults();
    m_LauncherMotorRight.restoreFactoryDefaults();
    
    /*    Ramp Rates (Open & Closed)
    * Time in seconds that it would take the controller to go from zero
    * to full throttle.
    */
    m_LauncherMotorLeft.setOpenLoopRampRate(Constants.Launcher.kOpenRampRate); // Zero to Full Throttle
    m_LauncherMotorLeft.setClosedLoopRampRate(Constants.Launcher.kClosedRampRate); // Zero to Full Throttle
    m_LauncherMotorRight.setOpenLoopRampRate(Constants.Launcher.kOpenRampRate); // Zero to Full Throttle
    m_LauncherMotorRight.setClosedLoopRampRate(Constants.Launcher.kClosedRampRate); // Zero to Full Throttle
    
    /*    Motor Idle Mode (Brake or Coast) */
    m_LauncherMotorLeft.setIdleMode(Constants.Launcher.kIdleMode); // Brake or Coast
    m_LauncherMotorRight.setIdleMode(Constants.Launcher.kIdleMode); // Brake or Coast
    
    /*    Motor Command Inversion
     * Giving the motor a + command should result in green lights flashing
     * on the motor controller. Set the inversion to whatever gets you that.
     */
    m_LauncherMotorLeft.setInverted(Constants.Launcher.kLeftMotorInverted);
    m_LauncherMotorRight.setInverted(Constants.Launcher.kRightMotorInverted);
    
    /*    Current limit
     * NEO Brushless Motor has a low internal resistance, which can mean
     * large current spikes that could be enough to cause damage to the motor
     * and controller. This current limit provides a smarter strategy to deal
     * with high current draws and keep the motor and controller operating in
     * a safe region.
     */
    m_LauncherMotorLeft.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit); // 30 Amp Limit (40 Amp Breaker)
    m_LauncherMotorRight.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit); // 30 Amp Limit (40 Amp Breaker)

    /*    Set Follower
     * Set the right motor to follow the left motor because they will be doing
     * the same things anyway. It will be easier to just command one motor to do
     * something then it will be to command two at the same time.
     */
    m_LauncherMotorRight.follow(m_LauncherMotorLeft);

    /*    Relative Encoder
     * Create an encoder object for each motor that is made from the integrated
     * hall effect encoder. The encoder will be used to monitor and control the
     * speed in RPM of the motor.
     */
    m_EncoderLeft = m_LauncherMotorLeft.getEncoder();
    m_EncoderRight = m_LauncherMotorRight.getEncoder();

    m_EncoderLeft.setInverted(Constants.Launcher.kEncLeftInverted);
    m_EncoderRight.setInverted(Constants.Launcher.kEncRightInverted);

    m_EncoderLeft.setVelocityConversionFactor(Constants.Launcher.kEncVelConversion);
    m_EncoderRight.setVelocityConversionFactor(Constants.Launcher.kEncVelConversion);

    // Don't really care about position because the launcher only monitors/controls velocity
    m_EncoderLeft.setPosition(0.0);
    m_EncoderRight.setPosition(0.0);

    /*    PID Controller
     * Can use either the wpilib PIDController or the Spark PID controller. May be best
     * to use the Spark PID Controller just because we are trying to control Spark hardware
     * using the integrated Spark feedback device of the left motor since it is the master.
     */
    m_LauncherPIDController = m_LauncherMotorLeft.getPIDController();
    m_LauncherPIDController.setFeedbackDevice(m_EncoderLeft);

    /* Assign the local PID constants from the Constants file */
    kP = Constants.Launcher.kP; 
    kI = Constants.Launcher.kI;
    kD = Constants.Launcher.kD; 
    kIz = Constants.Launcher.kIz;
    kFF = Constants.Launcher.kFF;

    /* Set PID controller gains using local constants */
    m_LauncherPIDController.setP(kP);
    m_LauncherPIDController.setI(kI);
    m_LauncherPIDController.setD(kD);
    m_LauncherPIDController.setIZone(kIz);
    m_LauncherPIDController.setFF(kFF);
    m_LauncherPIDController.setOutputRange(Constants.Launcher.kMinOutput, Constants.Launcher.kMaxOutput);

    /* Put the inital PID Gains on the Dashboard so they can be tweaked */
    SmartDashboard.putNumber("Launcher kP", kP);
    SmartDashboard.putNumber("Launcher kI", kI);
    SmartDashboard.putNumber("Launcher kD", kD);
    SmartDashboard.putNumber("Launcher kI Zone", kIz);
    SmartDashboard.putNumber("Launcher kFeed Forward", kFF);

    configureCANStatusFrames(m_LauncherMotorLeft, 100, 20, 20, 0, 0, 0, 0);
    configureCANStatusFrames(m_LauncherMotorRight, 20, 20, 20, 0, 0, 0, 0);

    /* Save all the configurations to the motors */
    m_LauncherMotorLeft.burnFlash();
    m_LauncherMotorRight.burnFlash();
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
   * Stop motors from moving
   */
  public final void stop() {
    // Stop motors from running
    m_LauncherMotorLeft.stopMotor(); // Right motor follows left's lead
    // m_LauncherMotorRight.stopMotor();
  }


  
  /**
   * Load the note into the handler.
   * This is to pick the notes up from the floor or if we are good
   * at it, from the human player station!
   * The logic for stopping the intake if there is a note loaded will
   * be handled in the commad that is called for loading. To minimize
   * the amount of duplicate code, we will pass a speed to the function
   * that will either be slow for intaking the note or fast and quick
   * for moving the note out of the way of the wheels before spin up.
   * 
   * @param speed of the motor in percent output
   */
  public final void load(double speed) {
    m_LauncherMotorLeft.set(-speed); // Right motor follows left's lead
    // m_LauncherMotorRight.set(-speed);
  }


  
  /*
   * Launch the note. Launching is considered the wheels spinning in
   * such a way that the note is ejected from the robot. Left wheel
   * spinning CCW and right wheel spinning CW. The speed of the wheels
   * is dependent on the scoring position and is passed into the function
   * in terms of RPM so that the speed is more consistant time to time
   * then if it were a percent output.
   * 
   * The delays and solenoid energizing will occur as part of the
   * command to launch the note and not part of the launch function.
   * 
   * @param speed of the motor in RPM to set the setpoint of the PID controller to.
   */
  public final void setLaunchRPM (double speed) {
    m_LauncherPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity); // Right motor follows left's lead
    // m_LauncherMotorRight.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }


  
  /*
   * Launch the note. Launching is considered the wheels spinning in
   * such a way that the note is ejected from the robot. Left wheel
   * spinning CCW and right wheel spinning CW. 
   *
   * @param speed of the motor in percent output
   */
  public final void launch (double speed) {
    m_LauncherMotorLeft.set(speed); // Right motor follows left's lead
    // m_LauncherMotorRight.set(speed);
  }


  
  /**
   * Get the speed of the left launcher motor.
   * 
   * @return velocity of the left motor in RPM
   */
  public final double getVelocity() {
    return m_EncoderLeft.getVelocity();
  }


  
  /**
   * UPDATE PID CONTROLLER PARAMETERS
   * Using the values pulled from the smartdashboard, update the PID
   * parameters and reburn the controller flash. This will amke tuning
   * the gains much easier.
   */
  public final void updatePID() {
    /* Pull PID gains from the dashboard */
    double p = SmartDashboard.getNumber("Launcher kP", kP);
    double i = SmartDashboard.getNumber("Launcher kI", kI);
    double d = SmartDashboard.getNumber("Launcher kD", kD);
    double iz = SmartDashboard.getNumber("Launcher kI Zone", kIz);
    double ff = SmartDashboard.getNumber("Launcher kFeed Forward", kFF);

    /* If the gains changed, update the controller */
    if((kP != p)) { m_LauncherPIDController.setP(p); kP = p; }
    if((kI != i)) { m_LauncherPIDController.setI(i); kI = i; }
    if((kD != d)) { m_LauncherPIDController.setI(i); kD = d; }
    if((kIz != iz)) { m_LauncherPIDController.setI(i); kIz = iz; }
    if((kFF != ff)) { m_LauncherPIDController.setI(i); kFF = ff; }

    /* Reset the I gain accumulation amount */
    m_LauncherPIDController.setIAccum(0);

    /* Need to reburn the flash of the controller in order for the changes to take affect */
    m_LauncherMotorLeft.burnFlash();
    m_LauncherMotorRight.burnFlash();
  }


  
  /**
   * Great place put the smartdashboard output data. Includes live PID controller tuning
   * to make life a little easier.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* SMART DASHBOARD */
    SmartDashboard.putNumber("Launcher Motor Temp (F)", (m_LauncherMotorLeft.getMotorTemperature() * 9 / 5) + 32); // Default is C, Convert to F
    SmartDashboard.putNumber("Launcher Motor Current", m_LauncherMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Launcher Motor Input Voltage", m_LauncherMotorLeft.getBusVoltage());
    SmartDashboard.putNumber("Launcher Motor Duty Cycle", m_LauncherMotorLeft.getAppliedOutput());
    // SmartDashboard.putNumber("Launcher Position", getPosition()); Dont need this, position of a fly wheel is irrelevant
    SmartDashboard.putNumber("Launcher Velocity", getVelocity());
  }
}
