package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private final CANSparkMax m_ClimbMotor = new CANSparkMax(Constants.MotorIDs.kClimbMotorCANid, MotorType.kBrushless);
  private RelativeEncoder m_ClimbEncoder;
  private final DigitalInput m_LowProx = new DigitalInput(Constants.Climb.kProxLowerDIO);
  private final DigitalInput m_HighProx = new DigitalInput(Constants.Climb.kProxUpperDIO);

  public Climb() {
    /*
     * CLIMB MOTOR CONFIGURATION
     * All the following configuration parameters can be set in the Constants file.
     */
    m_ClimbMotor.restoreFactoryDefaults();
    
     /* Ramp Rates (Open & Closed)
      * Time in seconds that it would take the controller to go from zero
      * to full throttle.
      */
    m_ClimbMotor.setOpenLoopRampRate(Constants.Climb.kOpenRampRate); // Zero to Full Throttle
    m_ClimbMotor.setClosedLoopRampRate(Constants.Climb.kClosedRampRate); // Zero to Full Throttle
    
    /* Motor Idle Mode (Brake or Coast) */
    m_ClimbMotor.setIdleMode(Constants.Climb.kIdleMode); // Brake or Coast
    
    /* Motor Command Inversion
     * Giving the motor a + command should result in green lights flashing
     * on the motor controller. Set the inversion to whatever gets you that.
     */
    m_ClimbMotor.setInverted(Constants.Climb.kMotorInverted);
    
    /* Current limit
     * NEO Brushless Motor has a low internal resistance, which can mean
     * large current spikes that could be enough to cause damage to the motor
     * and controller. This current limit provides a smarter strategy to deal
     * with high current draws and keep the motor and controller operating in
     * a safe region.
     */
    m_ClimbMotor.setSmartCurrentLimit(Constants.Climb.kCurrentLimit); // 30 Amp Limit (40 Amp Breaker)

    /*
     * CLIMB ENCODER CONFIGURATION
     */
    m_ClimbEncoder = m_ClimbMotor.getEncoder();
    m_ClimbEncoder.setPositionConversionFactor(Constants.Climb.kEncPosConversion); // Inches
    m_ClimbEncoder.setPosition(Constants.Climb.kResetPosition);

    configureCANStatusFrames(m_ClimbMotor, 100, 20, 20, 20, 0, 0, 0);

    /* Save Climb Motor Settings to the Flash Memory of the Controller */
    m_ClimbMotor.burnFlash();//
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
   * CLIMBER APARATUS MOVING UPWARDS
   * This function will use a constant percent output set in
   * the constants file to set motor speed output.
   */
  public final void Up() {
    m_ClimbMotor.set(Constants.Climb.kSpeedUp);
  }

  /**
   * CLIMBER APARATUS MOVING DOWNWARDS
   * This function will use a constant percent output set in
   * the constants file to set motor speed output.
   */
  public final void Down() {
    m_ClimbMotor.set(-Constants.Climb.kSpeedDown);
  }

  /** 
   * CLIMBER APARATUS STOPPED
   * Stop all commands to the climber motor. This will leave the
   * motor in what ever idle mode is set in the configurations.
   */
  public final void Stop() {
    m_ClimbMotor.set(0.0);
  }
  
  /**
   * GET CLIMBER APARATUS POSITION
   * 
   * This can be either defaul counts or inches of the climber hook
   * depending on what the encoder conversion factor is set to.
   * Additionaly a zero position offset can be applied to get the
   * position above the ground rather than relative to the reset position.
   * 
   * @return motor position
   */
  public final double getPosition() {
    return (m_ClimbEncoder.getPosition() * 16 / 1072) + Constants.Climb.kResetPosition;
  }

  /**
   * RESET THE CLIMBER ENCODER POSITION
   * 
   * Resets the relative position of the climber encoder to eith zero
   * or an offset from the ground to get position relative to the floor.
   */
   public final void resetPosition() {
    m_ClimbEncoder.setPosition(Constants.Climb.kResetPosition);
   }
  
  /** 
   * GET STATE OF LOWER PROX SWITCH
   * Returns true when the prox switch is triggered by the climbing mechanism
   * which signifies that the climber is at it's minimum climbing height. If
   * the switch is triggered true, it will also zero the position of the encoder.
   * 
   * @return state of lower climb prox switch
   */
  public final boolean getLowerProx() {
    boolean state = !m_LowProx.get();
    if (state) {resetPosition();}
    return state;
   }
  
  /** 
   * GET STATE OF UPPER PROX SWITCH
   * Returns true when the prox switch is triggered by the climbing mechanism
   * which signifies that the climber is at it's max climbing height
   * 
   * @return state of upper climb prox switch
   */
  public final boolean getHigherProx() {
    return !m_HighProx.get();
   }

  public boolean atSpeakerHeight() {
    return (getPosition() < Constants.Climb.kSpeakerHeight + 1) && (getPosition() > Constants.Climb.kSpeakerHeight - .1);
  }

  public boolean atStageHeight() {
    return (getPosition() < 26.9);
  }
  
  @Override
  public void periodic() {
    /* MAIN SMART DASHBOARD */
    SmartDashboard.putBoolean("Climb Low Switch", getLowerProx());
    SmartDashboard.putBoolean("Climb High Switch", getHigherProx());
    SmartDashboard.putNumber("Climb Position", getPosition());
    
    /* TROUBLESHOOT SMART DASHBOARD */
    // SmartDashboard.putNumber("Climb Motor Temp (F)", (m_ClimbMotor.getMotorTemperature() * 9 / 5) + 32); // Default is C, Convert to F
    // SmartDashboard.putNumber("Climb Motor Current", m_ClimbMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Climb Motor Input Voltage", m_ClimbMotor.getBusVoltage());
    // SmartDashboard.putNumber("Climb Motor Duty Cycle", m_ClimbMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Climb Velocity", getVelocity());
  }
}
