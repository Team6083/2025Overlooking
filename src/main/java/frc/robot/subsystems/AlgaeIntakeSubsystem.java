// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstant;
import frc.robot.lib.PowerDistribution;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSP intakeMotor;
  private final VictorSP rotateIntakeMotor;
  private final PIDController algaeRotatePID;
  private final PIDController algaeFrontPID;
  private final Encoder algaeFrontEncoder;
  private final Encoder algaeRotateEncoder;
  private final PowerDistribution powerDistribution;

  public AlgaeIntakeSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;
    algaeRotatePID = new PIDController(0, 0, 0);
    algaeFrontPID = new PIDController(0, 0, 0);
    intakeMotor = new VictorSP(AlgaeIntakeConstant.kIntakeMotorChannel);
    rotateIntakeMotor = new VictorSP(AlgaeIntakeConstant.kIntakeRotateMotorChannal);
    intakeMotor.setInverted(AlgaeIntakeConstant.kIntakeMotorInverted);
    algaeFrontEncoder = new Encoder(0, 1);
    algaeRotateEncoder = new Encoder(0, 0);
  }

  public void setIntakeMotor() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.setVoltage(0);

    }
    intakeMotor.setVoltage(AlgaeIntakeConstant.kIntakeVoltage);
  }

  public void setReIntake() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.setVoltage(0);

    }
    intakeMotor.setVoltage(AlgaeIntakeConstant.kReIntakeVoltage);
  }

  public void stopIntakeMotor() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.setVoltage(0);

    }
    intakeMotor.setVoltage(0);
  }

  public double getRotateIntakeSetpoint() {
    return algaeRotatePID.getSetpoint();
  }

  public double getFrontIntakeSetpoint() {
    return algaeFrontPID.getSetpoint();
  }

  public void setRotateIntakeSetpoint() {
    algaeRotatePID.setSetpoint(AlgaeIntakeConstant.kUpIntakeSetpoint);
  }

  public void setFrontIntakeSetpoint() {
    algaeFrontPID.setSetpoint(AlgaeIntakeConstant.kDownIntakeSetpoint);
  }

  public void setRotateIntake() {
    if (powerDistribution.isAlgaeRotateOverCurrent()) {
      rotateIntakeMotor.setVoltage(0);

    }
    rotateIntakeMotor.set(algaeRotatePID.calculate(algaeRotateEncoder.get()));
  }

  public void setFrontIntake() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.setVoltage(0);

    }
    intakeMotor.set(algaeFrontPID.calculate(algaeFrontEncoder.get()));
  }

  public void stopRotateIntakeMotor() {
    if (powerDistribution.isAlgaeRotateOverCurrent()) {
      rotateIntakeMotor.setVoltage(0);

    }
    rotateIntakeMotor.setVoltage(0);
  }

  public double getIntakeMotorRotate() {
    return algaeRotateEncoder.get();
  }

  public double getIntakeShooter() {
    return algaeFrontEncoder.get();
  }

  public Command setUpIntakeCmd() {
    Command cmd = runEnd(this::setRotateIntake, this::stopRotateIntakeMotor);
    return cmd;
  }

  public Command setDownIntakeCmd() {
    Command cmd = runEnd(this::setFrontIntake, this::stopRotateIntakeMotor);
    return cmd;
  }

  public Command setIntakeMotorCmd() { // 吸入 algae 的 cmd
    Command cmd = runEnd(this::setIntakeMotor, this::stopIntakeMotor);
    return cmd;
  }

  public Command setReIntakeMotorCmd() { // 吐出 algae 的 cmd
    Command cmd = runEnd(this::setIntakeMotor, this::stopIntakeMotor);
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
