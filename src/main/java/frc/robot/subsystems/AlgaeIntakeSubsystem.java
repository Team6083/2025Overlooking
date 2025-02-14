// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstant;
import frc.robot.lib.PowerDistribution;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSPX intakeMotor;
  private final VictorSPX rotateIntakeMotor;
  private final PIDController algaeRotatePID;
  private final PIDController algaeFrontPID;
  private final Encoder algaeFrontEncoder;
  private final Encoder algaeRotateEncoder;
  private final PowerDistribution powerDistribution;

  public AlgaeIntakeSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;
    algaeRotatePID = new PIDController(0, 0, 0);
    algaeFrontPID = new PIDController(0, 0, 0);
    intakeMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeMotorChannel);
    rotateIntakeMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeRotateMotorChannal);
    intakeMotor.setInverted(AlgaeIntakeConstant.kIntakeMotorInverted);
    algaeFrontEncoder = new Encoder(0, 1);
    algaeRotateEncoder = new Encoder(0, 0);
    algaeRotateEncoder.setDistancePerPulse(360.0 / 2048);
  }

  public void setIntakeMotor() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);

    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 1);
  }

  public void setReIntake() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);

    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 1);
  }

  public void stopIntakeMotor() {

    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void setUpRotateIntakeSetpoint() {
    algaeRotatePID.setSetpoint(AlgaeIntakeConstant.kUprotateIntakeSetpoint);
  }

  public void setDownRotateIntakeSetpoint() {
    algaeRotatePID.setSetpoint(AlgaeIntakeConstant.kDownrotateIntakeSetpoint);
  }

  private double getFrontIntakeSpeed() { // 用來測量轉速
    algaeFrontPID.setSetpoint(AlgaeIntakeConstant.kfrontIntakeSetpoint);
    return algaeFrontEncoder.getRate();

  }

  public void stopRotateIntake() {
    rotateIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public double getIntakeMotorRotate() {
    return algaeRotateEncoder.getDistance();
  }

  public double getIntakeShooter() {
    return algaeFrontEncoder.getRate();
  }

  public Command getFrontIntakeSpeedCmd() {
    Command cmd = runEnd(this::getFrontIntakeSpeed, this::stopIntakeMotor);
    return cmd;
  }

  public Command setIntakeMotorCmd() { // 吸入 algae 的 cmd
    Command cmd = runEnd(this::setIntakeMotor, this::stopIntakeMotor);
    return cmd;
  }

  public Command setReIntakeMotorCmd() { // 吐出 algae 的 cmd
    Command cmd = runEnd(this::setReIntake, this::stopIntakeMotor);
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (powerDistribution.isAlgaeRotateOverCurrent()) {
      rotateIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);

    }
    double output = algaeRotatePID.calculate(algaeRotateEncoder.get() / 2048);
    rotateIntakeMotor.set(ControlMode.PercentOutput, output);

  }
}
