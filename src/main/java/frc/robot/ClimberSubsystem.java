// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
private final VictorSPX climbermotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  climbermotor = new VictorSPX(43);
  }

  private void climbUp (){
  
  }

  private void climbDown(){
  climbermotor.set(-0.35);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
