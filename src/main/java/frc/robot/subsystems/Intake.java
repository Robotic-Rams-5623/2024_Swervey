package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.MotorIDs.kClimbMotorCANid, MotorType.kBrushless);
  private final DigitalInput m_NoteProx = new DigitalInput(Constants.Intake.kIntakeProxDIport);
  

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
