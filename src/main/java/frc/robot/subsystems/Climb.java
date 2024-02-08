// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private final CANSparkMax m_ClimbMotor = new CANSparkMax(Constants.MotorIDs.kClimbMotorCANid, MotorType.kBrushless);
  private final RelativeEncoder m_ClimbEncoder;

  public Climb() {
    m_ClimbMotor.restoreFactoryDefaults(); //

    m_ClimbEncoder = m_ClimbMotor.getEncoder();
    m_ClimbEncoder.setInverted(false);
    
    m_ClimbMotor.burnFlash();//
  }

  public final void Up() {}

  public final void Down() {}

  public final void Stop() {}
  
  public final double getPosition() {
    return 0.0;
  }

 public final void resetPosition() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
