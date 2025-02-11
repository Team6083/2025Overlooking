// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstant;
import frc.robot.lib.PowerDistribution;

public class ClimberSubsystem extends SubsystemBase {
  private final VictorSP climberMotor;
  private final PIDController climberPID;
  private final Encoder climberEncoder;
  private final PowerDistribution powerDistribution;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;
    climberMotor = new VictorSP(43);
    climberPID = new PIDController(0.08, 0, 0);
    climberEncoder = new Encoder(0, 0);
    climberMotor.setInverted(ClimberConstant.kclimberMotorInverted);
    climberEncoder.setReverseDirection(ClimberConstant.kclimberEncoderInverted);
  }

  private void climbUp() {
    if (powerDistribution.isClimberOverCurrent()) {
      climberMotor.set(0);
      return;
    }
    climberMotor.set(climberPID.calculate(climberEncoder.get()));
  }

  private void setSetpoint() {
    climberPID.setSetpoint(ClimberConstant.kClimberSetpoint);
  }

  private void climbDown() {
    if (powerDistribution.isClimberOverCurrent()) {
      climberMotor.set(0);
      return;
    }
    climberMotor.set(ClimberConstant.kClimbDownSpeed);
  }

  private double getSetpoint() {
    return climberPID.getSetpoint();
  }

  private double getClimberRate() {
    return climberEncoder.get();
  }

  public Command climbUpCmd() {
    Command cmd = run(this::climbUp);
    return cmd;
  }

  public Command climbDownCmd() {
    Command cmd = run(this::climbDown);
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
