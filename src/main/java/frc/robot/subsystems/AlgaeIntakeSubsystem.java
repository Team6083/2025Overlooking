// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeIntakeConstant;
import frc.robot.lib.PowerDistribution;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSPX intakeMotor;
  private final VictorSPX rotateIntakeMotor;
  private final PIDController algaeRotatePID;
  private final Encoder algaeRotateEncoder;
  private final PowerDistribution powerDistribution;

  public AlgaeIntakeSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;
    algaeRotatePID = new PIDController(
        AlgaeIntakeConstant.rotMotorPIDkD,
        AlgaeIntakeConstant.rotMotorPIDkI,
        AlgaeIntakeConstant.rotMotorPIDkD);

    intakeMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeMotorChannel);
    rotateIntakeMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeRotateMotorChannal);

    intakeMotor.setInverted(AlgaeIntakeConstant.kIntakeMotorInverted);
    rotateIntakeMotor.setInverted(AlgaeIntakeConstant.kRotateIntakeMotorInverted);

    algaeRotateEncoder = new Encoder(AlgaeIntakeConstant.kAlgaeEncoderChannelA,
        AlgaeIntakeConstant.kAlgaeEncoderChannelB);
    algaeRotateEncoder.setDistancePerPulse(AlgaeIntakeConstant.kDistancePerPulse);
  }

  public void setIntakeMotorFastOn() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput,
        AlgaeIntakeConstant.kIntakeFastSpeed);
  }

  public void setIntakeSlowOn() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput,
        AlgaeIntakeConstant.kIntakeSlowSpeed);
  }

  public void setReIntake() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput,
        AlgaeIntakeConstant.kReIntakeSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public Boolean isOverLimit() {
    return powerDistribution.algaeIntakeCurrent() <= AlgaeIntakeConstant.kIntakeCurrentLimit;
  }

  public void setUpRotateIntakeSetpoint() {
    algaeRotatePID.setSetpoint(AlgaeIntakeConstant.kUpRotateIntakeSetpoint);
  }

  public void setDownRotateIntakeSetpoint() {
    algaeRotatePID.setSetpoint(AlgaeIntakeConstant.kDownRotateIntakeSetpoint);
  }

  public void stopRotateIntake() {
    rotateIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public double getIntakeMotorRotate() {
    return algaeRotateEncoder.getDistance() + AlgaeIntakeConstant.kRotateEncoderOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (powerDistribution.isAlgaeRotateOverCurrent()) {
      rotateIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    double output = algaeRotatePID.calculate(getIntakeMotorRotate());
    rotateIntakeMotor.set(ControlMode.PercentOutput, output);

    SmartDashboard.putNumber("algaeRotateSetpoint", algaeRotatePID.getSetpoint());
    SmartDashboard.putNumber("algaeRotateDistance", algaeRotateEncoder.getDistance());
    SmartDashboard.putNumber("algaeRotateCurrent", getIntakeMotorRotate());
    SmartDashboard.putNumber("algaeIntakeVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("algaeRotateVoltage", rotateIntakeMotor.getMotorOutputVoltage());
    SmartDashboard.putBoolean("isOverLimit", isOverLimit());
  }

  public Command intakeCmd(double intakeSpeed) {
    Command cmd;
    if (intakeSpeed > 0.8) {
      cmd = runEnd(this::setIntakeMotorFastOn, this::stopIntakeMotor);
    } else {
      cmd = runEnd(this::setIntakeSlowOn, this::stopIntakeMotor);
    }
    cmd.setName("setIntakeCmd");
    return cmd;
  }

  public Command autoIntakeCmd() { // 吸入 algae 的 cmd
    Command cmd = run(this::setIntakeMotorFastOn)
        .until(this::isOverLimit)
        .andThen(this.runEnd(this::setIntakeSlowOn, this::stopIntakeMotor));
    cmd.setName("setAutoIntakeCmd");
    return cmd;
  }

  public Command reIntakeCmd() { // 吐出 algae 的 cmd
    Command cmd = runEnd(this::setReIntake, this::stopIntakeMotor);
    cmd.setName("setReIntakeMotorCmd");
    return cmd;
  }

  public Command upRotatePIDCmd() {
    Command cmd = run(this::setUpRotateIntakeSetpoint);
    cmd.setName("setUpRotateIntakeSetpoint");
    return cmd;
  }

  public Command downRotatePIDCmd() {
    Command cmd = run(this::setDownRotateIntakeSetpoint);
    cmd.setName("setDownRotateIntakeSetpoint");
    return cmd;
  }
}
