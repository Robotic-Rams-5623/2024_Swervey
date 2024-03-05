package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Solenoid extends SubsystemBase {
  /* Create a New Solenoid Object:
   * Since the solenoid we are using to feed the note into the launch wheels is a 12V
   * solneoid and we don't want to over complicate the circuitry with a relay controlled
   * by the RoboRIO DIO ports, lets run the solenoid off of the switchable port on the PDH!
   * The solenoid was moved into it's own subsystem seperate of the launcher and handler
   * because it was causing interupts in controlling of the other systems. To prevent the
   * handler or launcher from stopping when the solenoid was called, it was completely
   * seperated out.
   * 
   * IMPORTANT: REV ROBOTICS POWER ISTRIBUTION HUB (PDH) NEEDS TO HAVE CAN ID OF 1
   */
  private final PowerDistribution m_PDH = new PowerDistribution(1, ModuleType.kRev);

  public Solenoid() {
    // THe PDH automatically enables the switchable port on boot. To prevent the solenoid
    // from draining all the power, turn it off as soon as possible!
    feedRetract();
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
}
