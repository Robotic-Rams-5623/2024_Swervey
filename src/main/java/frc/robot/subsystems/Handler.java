package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Handler extends SubsystemBase {
  /** Creates a new Intake. */
  private final DigitalInput m_NoteProx = new DigitalInput(Constants.Intake.kIntakeProxDIport);
  

  public Handler() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
