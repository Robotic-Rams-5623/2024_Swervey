// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.MotorIDs.kIntakeMotorCANid, MotorType.kBrushless);
  private final RelativeEncoder m_IntakeEncoder;
  private final DigitalInput m_NoteProx = new DigitalInput(Constants.Intake.kIntakeProxDIport);

  public Intake() {
    /*
     * Intake MOTOR CONFIGURATION
     * All the following configuration parameters can be set in the Constants file.
     */
    m_IntakeMotor.restoreFactoryDefaults();
    
     /* Ramp Rates (Open & Closed)
      * Time in seconds that it would take the controller to go from zero
      * to full throttle.
      */
    m_IntakeMotor.setOpenLoopRampRate(Constants.Intake.kOpenRampRate); // Zero to Full Throttle
    m_IntakeMotor.setClosedLoopRampRate(Constants.Intake.kClosedRampRate); // Zero to Full Throttle
    
    /* Motor Idle Mode (Brake or Coast) */
    m_IntakeMotor.setIdleMode(Constants.Intake.kIdleMode); // Brake or Coast
    
    /* Motor Command Inversion
     * Giving the motor a + command should result in green lights flashing
     * on the motor controller. Set the inversion to whatever gets you that.
     */
    m_IntakeMotor.setInverted(Constants.Intake.kMotorInverted);
    
    /* Current limit
     * NEO Brushless Motor has a low internal resistance, which can mean
     * large current spikes that could be enough to cause damage to the motor
     * and controller. This current limit provides a smarter strategy to deal
     * with high current draws and keep the motor and controller operating in
     * a safe region.
     */
    m_IntakeMotor.setSmartCurrentLimit(Constants.Intake.kCurrentLimit); // 30 Amp Limit (40 Amp Breaker)

    configureCANStatusFrames(m_IntakeMotor, 100, 20, 20, 0, 0, 0, 0);

    m_IntakeEncoder = m_IntakeMotor.getEncoder(Type.kHallSensor, 42);

    /* Save Intake Motor Settings to the Flash Memory of the Controller */
    m_IntakeMotor.burnFlash();//
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

  public void In() {
    // If there is no note in the handler or the overide flag is true, then spin the intake
    // if (!getNoteProx() || override) {
    //   m_IntakeMotor.set(-Constants.Intake.kSpeedIn);
    // } else {
    //   Stop();
    // }
    m_IntakeMotor.set(-Constants.Intake.kSpeedIn);
  }

  public void Out() {
    m_IntakeMotor.set(-Constants.Intake.kSpeedOut);
  }

  public void Stop() {
    m_IntakeMotor.set(0.0);
  }

  public boolean getNoteProx() {
    return m_NoteProx.get();
  }

  public double getVelocity() {
    return m_IntakeEncoder.getVelocity();
  }

  public boolean isMoving() {
    return (getVelocity() > 20);
  }




  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note In Robot", getNoteProx());
    SmartDashboard.putBoolean("Intake Moving", isMoving());
  }
}
