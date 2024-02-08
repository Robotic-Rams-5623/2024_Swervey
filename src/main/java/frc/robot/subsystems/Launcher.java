// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  
private final CANSparkMax m_LauncherMotor = new CANSparkMax(Constants.MotorIDs.kRLauncherMotorCANid, MotorType.kBrushless);

  public Launcher() {

  }

  // Funtions


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
