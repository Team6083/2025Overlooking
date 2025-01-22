// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final VictorSP climbermotor;
  private final PIDController climberPID;
  private final Encoder climberEncoder;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climbermotor = new VictorSP(43);
    climberPID = new PIDController(0.08, 0, 0);
    climberEncoder = new Encoder(0, 0);
  }

  private void climbUp() {
    climbermotor.set(climberPID.calculate(climberEncoder.get()));
  }

  private void setsetpoint() {
    climberPID.setSetpoint(1);
  }

  private void climbDown() {
    climbermotor.set(-0.35);
  }

  private double getSetpoint() {
    return climberPID.getSetpoint();
  }

  public Command climbUpcmd() {
    Command cmd = run(this::climbUp);
    return cmd;
  }

  public Command climbDowncmd() {
    Command cmd = run(this::climbDown);
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
