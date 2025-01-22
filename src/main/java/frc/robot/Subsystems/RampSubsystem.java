// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RampSubsystem extends SubsystemBase {
 VictorSP rampmotor = new VictorSP(0);
  public RampSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  void rampstrite (){
    rampmotor.set(1);
  }
  void rampback (){
    rampmotor.set(-1);
  }
  public Command rampstritecmd (){
    Command cmd = run(this::rampstritecmd);
    return cmd;
  }
  public Command rampbackcmd (){
    Command cmd = run(this::rampbackcmd);
    return cmd;
  }
}
