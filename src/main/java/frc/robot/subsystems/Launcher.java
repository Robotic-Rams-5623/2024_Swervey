// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  
private final CANSparkMax m_LauncherMotorLeft = new CANSparkMax(Constants.MotorIDs.kLLauncherMotorCANid, MotorType.kBrushless);
private final CANSparkMax m_LauncherMotorRight = new CANSparkMax(Constants.MotorIDs.kRLauncherMotorCANid, MotorType.kBrushless);

/* The servo is really just a electro actuator. Need to find a way to give it 12V
 * Most likely from the switchable port on the PDH using a 5 to 10 amp fuse. */
// private final Servo m_LauncherServo = new Servo(Constants.Launcher.kLauncherServoid);
private final PowerDistribution m_powerHub = new PowerDistribution(0, ModuleType.kRev); // PROBABLY PUT THIS IN THE hANDLER SUBSYSTEM

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
    m_LauncherMotorLeft.setInverted(Constants.Launcher.kMotorInverted);
    m_LauncherMotorRight.setInverted(Constants.Launcher.kMotorInverted);
    
    /*    Current limit
     * NEO Brushless Motor has a low internal resistance, which can mean
     * large current spikes that could be enough to cause damage to the motor
     * and controller. This current limit provides a smarter strategy to deal
     * with high current draws and keep the motor and controller operating in
     * a safe region.
     */
    m_LauncherMotorLeft.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit); // 30 Amp Limit (40 Amp Breaker)
    m_LauncherMotorRight.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit); // 30 Amp Limit (40 Amp Breaker)

    m_LauncherMotorLeft.burnFlash();
    m_LauncherMotorRight.burnFlash();
  }

  /**
   * Stop motors from moving
   */
  public final void Stop() {
    // Stop motors from running
    m_LauncherMotorLeft.stopMotor();
    m_LauncherMotorRight.stopMotor();
  }

  public final void Retract() {
    // Unsure about intended usage???
  }

  public final void Extend() {
    // Unsure about intended usage???
  }

  /**
   * Load the note into the handler.
   * This is to pick the notes up from the floor or if we are good
   * at it, from the human player station!
   * Whether or not the intake will move is dependent on the state of
   * the note prox sensor which can be fed through the function as a
   * parameter or overrided in the command call if needed (note is stuck).
   * When launching the note we want to perform the same
   */
  public final void Load(double speed) {
    m_LauncherMotorLeft.set(-Constants.Launcher.kSpeedPushLowRPM);
    m_LauncherMotorRight.set(-Constants.Launcher.kSpeedPushLowRPM);
  }

  /*
   * Launch the note. Launching is considered the wheels spinning in
   * such a way that the note is ejected from the robot. Left wheel
   * spinning CCW and right wheel spinning CW. The speed of the wheels
   * 
   * The delays and solenoid energizing will occur as part of the
   * command to launch the note and not part of the launch function.
   */
  public final void Launch() {
    m_LauncherMotorLeft.set(Constants.Launcher.kSpeedPush);
    m_LauncherMotorRight.set(Constants.Launcher.kSpeedPush);
  }




  public final double GetSpeed() {
    // Return current speed of motors

    double launcher_speed = 0.0;
    return launcher_speed;
  }

  /*  !! Don't know real implementation !!
  * public final boolean ReadProxSensor():
  *  return bool(m_LauncherFullSensor.read())
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
